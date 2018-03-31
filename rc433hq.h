#pragma once

#if defined(ARDUINO)
#	include <Arduino.h>
#else
#	include <stdint.h>
#	define byte uint8_t
#	define word uint16_t
#endif // defined(ARDUINO)

/**
  @file rc433hq.h
*/

/** \brief Data handler interface responsible for decoding of the binary data into information packets
 */
class IRC433DataReceiver {
public:
	virtual void HandleData(const byte *data, word bits) = 0;
	
};

/** \brief Line protocol interface responsible for decoding of the pulses into the binary data
 */
class IRC433PulseDecoder {
public:
	virtual void HandleEdge(unsigned long time, bool direction) = 0;
};

/** \brief NoiseFilter implements the IRC433PulseDecoder, eliminates fast edge changes from the data and forwards (slightly delayed) edges into the connected decoder
 */
class NoiseFilter: public IRC433PulseDecoder {
private:
	IRC433PulseDecoder &decoder;
	unsigned long minPulseDuration;
	bool lastEdgeValid;
	unsigned long lastEdgeTime;
	bool lastEdgeDirection;
public:
	NoiseFilter(IRC433PulseDecoder &adecoder, unsigned long aminPulseDuration):
		decoder(adecoder), 
		minPulseDuration(aminPulseDuration), 
		lastEdgeValid(false) 
	{
	}
	virtual void HandleEdge(unsigned long time, bool direction);
};


/** \brief Basic sync protocol decoder
 */
/*
class RC433BasicSyncPulseDecoder: public IRC433PulseDecoder {
private:
	IRC433DataReceiver &dataReceiver;
	word syncFirstUs, syncSecondUs, zeroFirstUs, zeroSecondUs, oneFirstUs, oneSecondUs;
	bool highFirst;
	word minBits, maxBits;
	
public:	
	RC433BasicSyncPulseDecoder(IRC433DataReceiver &adataReceiver, word asyncFirstUs, word asyncSecondUs, word azeroFirstUs, word azeroSecondUs, word aoneFirstUs, word aoneSecondUs, bool ahighFirst, word aminBits, word amaxBits):
		dataReceiver(adataReceiver),
		syncFirstUs(asyncFirstUs),
		syncSecondUs(asyncSecondUs), 
		zeroFirstUs(azeroFirstUs), 
		zeroSecondUs(azeroSecondUs), 
		oneFirstUs(aoneFirstUs), 
		oneSecondUs(aoneSecondUs), 
		highFirst(ahighFirst),
		minBits(aminBits), maxBits(amaxBits)
	{}
	
	virtual void HandleEdge(unsigned long time, bool drection);
};
*/
