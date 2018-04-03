#pragma once

#define DEBUG

#if defined(ARDUINO)
#	include <Arduino.h>
#else
#	include <stdint.h>
#	include <stddef.h>
#	include <math.h>

#	define byte uint8_t
#	define word uint16_t

	// fake definitions, we are not touching HW
#	define digitalPinToInterrupt(pin) 0
#	define CHANGE 0
#	define INPUT_PULLUP 0
#	define HIGH 0
#	define LOW 0
#	define noInterrupts()
#	define interrupts()
#	define attachInterrupt(num, ptr, mode)
#	define detachInterrupt(num)
#	define pinMode(pin, mode)
#	define digitalRead(pin) 0
#	define digitalWrite(pin, value)
#	define micros() 0
#	define pinMode(pin, mode)

#endif // defined(ARDUINO)

#ifdef DEBUG
#	define LOG_MESSAGE(message) LogMessage(message)
#else
#	define LOG_MESSAGE(message)
#endif

/**
  @file rc433hq.h
*/

typedef unsigned long RC433HQMicroseconds;

//////////////////////////////////////////////////////////////////////////////////
// IRC433Logger declaration
//////////////////////////////////////////////////////////////////////////////////

/** \brief Logger interface responsible writing log messages
 */
class IRC433Logger {
public:
	virtual void LogMessage(const char *message) = 0;
	
};


//////////////////////////////////////////////////////////////////////////////////
// IRC433DataReceiver declaration
//////////////////////////////////////////////////////////////////////////////////

/** \brief Data handler interface responsible for decoding of the binary data into information packets
 */
class IRC433DataReceiver {
public:
	virtual void HandleData(RC433HQMicroseconds time, const byte *data, size_t bits, double quality) = 0;
	
};


//////////////////////////////////////////////////////////////////////////////////
// IRC433PulseProcessor declaration
//////////////////////////////////////////////////////////////////////////////////

/** \brief Line protocol interface responsible for decoding of the pulses into the binary data
 */
class IRC433PulseProcessor {
public:
	// handle one rising or falling edge that was detected at specified time
	virtual void HandleEdge(RC433HQMicroseconds time, bool direction) = 0;

	// used to notifi the processor that some edges could not be handled in the earlier stages (forgotten when buffers were full)
	virtual void HandleMissedEdges()
	{
	}
};


//////////////////////////////////////////////////////////////////////////////////
// RC433HQDataTransmitterBase declaration
//////////////////////////////////////////////////////////////////////////////////

// Interface for the transmitter that accepts the endoded data to be send via the HW layer
class RC433HQDataTransmitterBase {
public:

	// transmit the rising or falling edge and plan waiting after it
	virtual void TransmitEdge(bool direction, RC433HQMicroseconds duration) = 0;

	// transmit the pulse starting with rising edge
	void TransmitPulse(RC433HQMicroseconds highDuration, RC433HQMicroseconds lowDuration)
	{
		TransmitEdge(true, highDuration);
		TransmitEdge(false, lowDuration);
	}

	// transmit the repeated pulses
	void TransmitPulses(RC433HQMicroseconds highDuration, RC433HQMicroseconds lowDuration, size_t count)
	{
		for (size_t i = 0; i < count; i++) {
			TransmitPulse(highDuration, lowDuration);
		}
	}

};

//////////////////////////////////////////////////////////////////////////////////
// RC433PulseSignalSplitter declaration
//////////////////////////////////////////////////////////////////////////////////

/** \brief Signal splitter allows two proessors to be connected to a single signal source
 */
class RC433PulseSignalSplitter: public IRC433PulseProcessor {
private:
	IRC433PulseProcessor &first;
	IRC433PulseProcessor &second;
public:
	RC433PulseSignalSplitter(IRC433PulseProcessor &afirst, IRC433PulseProcessor &asecond):
		first(afirst),
		second(asecond)
	{
	}
	
	virtual void HandleEdge(RC433HQMicroseconds time, bool direction)
	{
		// handle the edge in both attached processors
		first.HandleEdge(time, direction);
		second.HandleEdge(time, direction);
	}

	virtual void HandleMissedEdges()
	{
		// inform both attached processors
		first.HandleMissedEdges();
		second.HandleMissedEdges();
	}
};


//////////////////////////////////////////////////////////////////////////////////
// RC433HQPulseBuffer declaration
//////////////////////////////////////////////////////////////////////////////////

/** \brief PulseBuffer implements the IRC433PulseProcessor, minimizes the time in the interrupt and passes the buffered data into the connected pulse decoder from the Process() method that needs to be periodically called from the loop
 */
class RC433HQPulseBuffer: public IRC433PulseProcessor {
private:
	// we will store word values into the buffer, while:
	//  - the highest bit 0x8000 means the direction of the edge (0x8000 is rising and 0x0000 is falling)
	//  - the lower 15 bits represent the time delay since the previous value in microseconds. A special value
	//    0x7fff is reserved and means that the next two words store the absolut time in microseconds.
	typedef word BufferValue;

private:
	IRC433PulseProcessor &connectedPulseDecoder;
	IRC433Logger *logger;
	size_t bufferSize;
	BufferValue *buffer;
	RC433HQMicroseconds lastStoredEdgeTime;  // valid, if there is at least one edge in the buffer
	RC433HQMicroseconds lastSentEdgeTime;    // initialized to 0, otherwise valid
	size_t dataIndex;  
	size_t freeIndex;
	size_t usedCount;	// count of the items (not edges) already stored into the buffer
	size_t missedCount; // count of the missed items, that could not be stored into the buffer

public:
	RC433HQPulseBuffer(IRC433PulseProcessor &aconnectedPulseDecoder, size_t abufferSize);
	~RC433HQPulseBuffer();

	// call this regularly from the loop to process the buffered data. reportedUsedCount is set to the number of items processed 
	// in this call, reportedMissedCount is set to the number of items that could not be stored into the buffer from the interrupt
	// because the buffer was full.
	void ProcessData(size_t &reportedBufferUsedCount, size_t &reportedProcessedCount, size_t &reportedMissedCount);

	virtual void HandleEdge(RC433HQMicroseconds time, bool direction);

private:
	void StoreAbsolutTime(RC433HQMicroseconds time, BufferValue directionMask);
	size_t CalculateNext(size_t index);
};


//////////////////////////////////////////////////////////////////////////////////
// RC433HQNoiseFilter declaration
//////////////////////////////////////////////////////////////////////////////////

/** \brief RC433HQNoiseFilter implements the IRC433PulseProcessor, eliminates fast edge changes from the data and forwards (slightly delayed) edges into the connected decoder
 */
class RC433HQNoiseFilter: public IRC433PulseProcessor {
private:
	IRC433PulseProcessor &decoder;
	RC433HQMicroseconds minPulseDuration;
	bool lastEdgeValid;
	RC433HQMicroseconds lastEdgeTime;
	bool lastEdgeDirection;
public:
	RC433HQNoiseFilter(IRC433PulseProcessor &adecoder, RC433HQMicroseconds aminPulseDuration):
		decoder(adecoder), 
		minPulseDuration(aminPulseDuration), 
		lastEdgeValid(false) 
	{
	}

	virtual void HandleEdge(RC433HQMicroseconds time, bool direction);

	virtual void HandleMissedEdges();
};


//////////////////////////////////////////////////////////////////////////////////
// RC433HQBasicSyncPulseDecoder declaration
//////////////////////////////////////////////////////////////////////////////////

static const size_t RC433HQ_MAX_PULSE_BITS = 128;

/** \brief Basic sync protocol decoder
 */
class RC433HQBasicSyncPulseDecoder: public IRC433PulseProcessor {
private:
	IRC433DataReceiver &dataReceiver;
	IRC433Logger *logger;
	word syncFirstUs, syncSecondUs, zeroFirstUs, zeroSecondUs, oneFirstUs, oneSecondUs;
	word toleranceUs; // max tolerance of rising or falling edges timing in us
	bool highFirst;
	word minBits, maxBits;
	bool syncDetected;
	RC433HQMicroseconds syncTime; 
	byte receivedData[RC433HQ_MAX_PULSE_BITS / 8];
	size_t receivedBits;
	bool previousRisingEdge;
	RC433HQMicroseconds previousRisingEdgeTime;
	bool previousFallingEdge;
	RC433HQMicroseconds previousFallingEdgeTime;
	double deltaPowerSum;  // sum of delta^2 for individual edges
	
public:	
	RC433HQBasicSyncPulseDecoder(IRC433DataReceiver &adataReceiver, word asyncFirstUs, word asyncSecondUs, word azeroFirstUs, word azeroSecondUs, word aoneFirstUs, word aoneSecondUs, word atoleranceUs, bool ahighFirst, word aminBits, word amaxBits):
		dataReceiver(adataReceiver),
		logger(0),
		syncFirstUs(asyncFirstUs),
		syncSecondUs(asyncSecondUs), 
		zeroFirstUs(azeroFirstUs), 
		zeroSecondUs(azeroSecondUs), 
		oneFirstUs(aoneFirstUs), 
		oneSecondUs(aoneSecondUs), 
		toleranceUs(atoleranceUs),
		highFirst(ahighFirst),
		minBits(aminBits), maxBits(amaxBits),
		syncDetected(false),
		receivedBits(0),
		previousRisingEdge(false),
		previousFallingEdge(false)
	{
		ClearReceivedBits();
	}

	void SetLogger(IRC433Logger &alogger)
	{ 
		logger = &alogger;
	}

	void LogMessage(const char *message)
	{ 
		if (logger) {
			logger->LogMessage(message);
		}
	}
	
	virtual void HandleEdge(RC433HQMicroseconds time, bool drection);

	virtual void HandleMissedEdges();

protected:
	// bits operations
	void ClearReceivedBits();
	void StoreReceivedBit(byte bit);

	// quality calculations
	void CalculateDelta(RC433HQMicroseconds expected, RC433HQMicroseconds actual);
	void ClearDelta();

	// sending of the cached data
	void SendReceivedData();
};


//////////////////////////////////////////////////////////////////////////////////
// RC433HQBasicSyncPulseEncoder declaration
//////////////////////////////////////////////////////////////////////////////////

class RC433HQBasicSyncPulseEncoder {
private:
	word syncFirstUs, syncSecondUs, zeroFirstUs, zeroSecondUs, oneFirstUs, oneSecondUs;
	bool highFirst;
	
public:	
	RC433HQBasicSyncPulseEncoder(word asyncFirstUs, word asyncSecondUs, word azeroFirstUs, word azeroSecondUs, word aoneFirstUs, word aoneSecondUs, bool ahighFirst):
		syncFirstUs(asyncFirstUs),
		syncSecondUs(asyncSecondUs), 
		zeroFirstUs(azeroFirstUs), 
		zeroSecondUs(azeroSecondUs), 
		oneFirstUs(aoneFirstUs), 
		oneSecondUs(aoneSecondUs), 
		highFirst(ahighFirst)
	{
	}

	// endode the data (sync + n data bits) and pass it to the transmitter
	void EncodeData(RC433HQDataTransmitterBase &transmitter, const byte *data, size_t bits, size_t repetitions);

};

//////////////////////////////////////////////////////////////////////////////////
// RC433HQReceiver declaration
//////////////////////////////////////////////////////////////////////////////////

class RC433HQReceiver {
private:
	static RC433HQReceiver *activeInstance;
private:
	IRC433PulseProcessor &decoder;
	bool iAmActiveInstance;
	int receiverGpioPin;

public:
	RC433HQReceiver(IRC433PulseProcessor &adecoder, int areceiverGpioPin);
	~RC433HQReceiver();

public:
	// disables receiving of the data (call before the transmission, if the reception of the transmitted data is not welcome)
	// Note: it's recommended to use an instance of class RC433HQReceptionDisabler for a temporary disabling and automated enabling
	void DisableReception();

	// enables the receiving of the data. By default the data reception is enabled.
	void EnableReception();

protected:
	// static interrupt handler - registered for the interrupt handling
	static void HandleInterrupt();

	// internal interrupt handler, called from the static method
	void HandleInterruptInternal();
};

// use an instance of this helper class to automatically disable and enable signal reception, when this class is being destroyed
// Usage:
//    // an instance of receiver
//    RC433HQReceiver receiver(...);
//    ...
//    {
//        // disable the reception
//        RC433HQReceptionDisabler disabler(receiver)
//
//        ... do something you want while the reception is disabled - typically transmit the data
//
//    } // reception is automatically enabled (you can never forget about this now!)
//
class RC433HQReceptionDisabler {
private:
	RC433HQReceiver &receiver;

public:
	RC433HQReceptionDisabler(RC433HQReceiver &areceiver):
		receiver(areceiver)
	{
		// disable the reception when the instance is created
		receiver.DisableReception();
	}
	~RC433HQReceptionDisabler()
	{
		// automatically re-enable reception, when this instance is being destroyed
		receiver.EnableReception();
	}
};

//////////////////////////////////////////////////////////////////////////////////
// RC433HQTransmitter declaration
//////////////////////////////////////////////////////////////////////////////////

// statistics stucture that is filled in by the transmitter
struct RC433HQTransmissionQualityStatistics {
	size_t countOfTransmittedEdges;	               // total count of transmitted edges
	size_t countOfDelayedEdges;                    // count of edges, where we missed the time of sending
	size_t countOfDelayedEdgesOutsideOfTolerance;  // count of edges, where we missed the time of sending
	double averageDelay;                           // average delay in microseconds calculated for the delayed edges
};

class RC433HQTransmitter: public RC433HQDataTransmitterBase {
private:
	int transmitterGpioPin;
	bool inTransitionMode;
	RC433HQMicroseconds maxDelayTolerance;
	RC433HQMicroseconds totalDelayedEdgesDelayTime;
	RC433HQTransmissionQualityStatistics *transmissionQualityStatistics;
	bool durationFinishTimeValid;             // specifies whether we are waiting for the duration of the previous edge
	RC433HQMicroseconds durationFinishTime;   // time in microseconds, when the duration of previous edge should finish

public:
	RC433HQTransmitter(int atransmitterGpioPin);
	~RC433HQTransmitter();

	// initializes the data transmission. Data can be sent only if the transmission is started
	void StartTransmission(RC433HQMicroseconds quietPeriodDurationBefore = 0, RC433HQMicroseconds amaxDelayTolerance = 0, RC433HQTransmissionQualityStatistics *atransmissionQualityStatistics = 0);

	// finalizes the data transmission
	void EndTransmission(RC433HQMicroseconds quietPeriodDurationAfter = 0);

	// transmit the rising or falling edge and plan waiting after it
	virtual void TransmitEdge(bool direction, RC433HQMicroseconds duration);

private:

	// report the delay of the transition of the edge into the statistics
	void ReportEdgeTransitionDelay(RC433HQMicroseconds delay);

	// plan wait for the duration
	void PlanWaitForDuration(RC433HQMicroseconds duration);

	// if there was a duration of the previous edge or quiet period, wait for it to finish
	void WaitForThePreviousDurationToFinish();

};
