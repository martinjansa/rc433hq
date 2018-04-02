#pragma once

//#define DEBUG

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
#	define digitalRead(pin) 0
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

// compares the two microseconds times with and returns true if equal withing a given tolerance
bool EqualWithTolerance(RC433HQMicroseconds a, RC433HQMicroseconds b, RC433HQMicroseconds tolerance);

/** \brief Logger interface responsible writing log messages
 */
class IRC433Logger {
public:
	virtual void LogMessage(const char *message) = 0;
	
};

/** \brief Data handler interface responsible for decoding of the binary data into information packets
 */
class IRC433DataReceiver {
public:
	virtual void HandleData(const byte *data, size_t bits, double quality) = 0;
	
};

/** \brief Line protocol interface responsible for decoding of the pulses into the binary data
 */
class IRC433PulseProcessor {
public:
	virtual void HandleEdge(RC433HQMicroseconds time, bool direction) = 0;
};

/** \brief PulseBuffer implements the IRC433PulseProcessor, minimizes the time in the interrupt and passes the buffered data into the connected pulse decoder from the Process() method that needs to be periodically called from the loop
 */
class RC433HQPulseBuffer: public IRC433PulseProcessor {
private:
	IRC433PulseProcessor &connectedPulseDecoder;
	IRC433Logger *logger;
	size_t bufferSize;
	RC433HQMicroseconds *times;
	bool *directions;
	size_t dataIndex;  
	size_t freeIndex;
	size_t usedCount;	// count of the items already stored into the buffer
	size_t missedCount; // count of the missed items, that could not be stored into the buffer

public:
	RC433HQPulseBuffer(IRC433PulseProcessor &aconnectedPulseDecoder, size_t abufferSize);
	~RC433HQPulseBuffer();

	// call this regularly from the loop to process the buffered data. reportedUsedCount is set to the number of items processed 
	// in this call, reportedMissedCount is set to the number of items that could not be stored into the buffer from the interrupt
	// because the buffer was full.
	void ProcessData(size_t &reportedUsedCount, size_t &reportedMissedCount);

	virtual void HandleEdge(RC433HQMicroseconds time, bool direction);

private:
	size_t CalculateNext(size_t index);
};

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
};

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

protected:
	// static interrupt handler - registered for the interrupt handling
	static void HandleInterrupt();

	// internal interrupt handler, called from the static method
	void HandleInterruptInternal();
};
