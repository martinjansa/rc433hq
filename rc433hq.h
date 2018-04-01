#pragma once

#if defined(ARDUINO)
#	include <Arduino.h>
#else
#	include <stdint.h>
#	include <stddef.h>
#	define byte uint8_t
#	define word uint16_t
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
	virtual void HandleData(const byte *data, size_t bits) = 0;
	
};

/** \brief Line protocol interface responsible for decoding of the pulses into the binary data
 */
class IRC433PulseDecoder {
public:
	virtual void HandleEdge(RC433HQMicroseconds time, bool direction) = 0;
};

/** \brief RC433HQNoiseFilter implements the IRC433PulseDecoder, eliminates fast edge changes from the data and forwards (slightly delayed) edges into the connected decoder
 */
class RC433HQNoiseFilter: public IRC433PulseDecoder {
private:
	IRC433PulseDecoder &decoder;
	RC433HQMicroseconds minPulseDuration;
	bool lastEdgeValid;
	RC433HQMicroseconds lastEdgeTime;
	bool lastEdgeDirection;
public:
	RC433HQNoiseFilter(IRC433PulseDecoder &adecoder, RC433HQMicroseconds aminPulseDuration):
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
class RC433HQBasicSyncPulseDecoder: public IRC433PulseDecoder {
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
	void ClearReceivedBits();
	void StoreReceivedBit(byte bit);
};
