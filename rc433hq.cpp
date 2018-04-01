#include "rc433hq.h"

#include <string.h>

bool EqualWithTolerance(RC433HQMicroseconds a, RC433HQMicroseconds b, RC433HQMicroseconds tolerance)
{
    return ((a - tolerance) <= b) && (b <= (a + tolerance));
}

void RC433HQNoiseFilter::HandleEdge(RC433HQMicroseconds time, bool direction)
{
    // if we have some edge in the memory
	if (lastEdgeValid) {

        // if this edge in the same direction as the last edge
        if (lastEdgeDirection == direction) {

            // ignore the first edge, keep the currect edge in the memory and quit
            lastEdgeTime = time;
            lastEdgeDirection = direction;
            lastEdgeValid = true;
            return;
        }

        // calculate the duration of the last pulse
        RC433HQMicroseconds duration = (time - lastEdgeTime);

        // if the last pulse was to short
        if (duration < minPulseDuration) {

            // ignore the last pulse and also the one currently obtained
            lastEdgeValid = false;
            return;
        }

        // send the last edge into the connected decoder
        decoder.HandleEdge(lastEdgeTime, lastEdgeDirection);
    }

    // keep the currect edge in the memory
    lastEdgeTime = time;
    lastEdgeDirection = direction;
    lastEdgeValid = true;
}


void RC433HQBasicSyncPulseDecoder::HandleEdge(RC433HQMicroseconds time, bool direction)
{
    // if the rising edge is being handled
    if (direction) {

        LOG_MESSAGE("Handling rising edge.\n");

        // if both last rising and falling edge were syncDetected
        if (previousRisingEdge && previousFallingEdge) {

            LOG_MESSAGE("Previous rising and falling edge were detected.\n");

            // calcuate the high and low durations
            RC433HQMicroseconds highDuration = previousFallingEdgeTime - previousRisingEdgeTime;
            RC433HQMicroseconds lowDuration = time - previousFallingEdgeTime;

            // if the last pulse represented a 1
            if (syncDetected && EqualWithTolerance(highDuration, oneFirstUs, toleranceUs) && EqualWithTolerance(lowDuration, oneSecondUs, toleranceUs)) {

                LOG_MESSAGE("Sync detected and pulse represents bit 1.\n");

                // store the bit
                StoreReceivedBit(1);

                // if we reached the higher limit of the received bit
                if (receivedBits == maxBits) {

                    LOG_MESSAGE("Max bits received, sending data.\n");

                    // send the data to the data receiver
                    dataReceiver.HandleData(receivedData, receivedBits);
                    syncDetected = false;
                    ClearReceivedBits();
                }

            } else {

                // if the last pulse represented a 1
                if (syncDetected && EqualWithTolerance(highDuration, zeroFirstUs, toleranceUs) && EqualWithTolerance(lowDuration, zeroSecondUs, toleranceUs)) {

                    LOG_MESSAGE("Sync detected and pulse represents bit 0.\n");

                    // store the bit
                    StoreReceivedBit(0);

                    // if we reached the higher limit of the received bit
                    if (receivedBits == maxBits) {

                        LOG_MESSAGE("Max bits received, sending data.\n");

                        // send the data to the data receiver
                        dataReceiver.HandleData(receivedData, receivedBits);
                        syncDetected = false;
                        ClearReceivedBits();
                    }

                } else {

                    // no data pulse was detected

                    // if some data were received before and is above the minimal length
                    if (receivedBits >= minBits) {

                        LOG_MESSAGE("Min bits received before non data pulse, sending data.\n");

                        // send the data to the data receiver
                        dataReceiver.HandleData(receivedData, receivedBits);
                        ClearReceivedBits();
                    }

                    // if the sync pulse was detected
                    if (EqualWithTolerance(highDuration, syncFirstUs, toleranceUs) && EqualWithTolerance(lowDuration, syncSecondUs, toleranceUs)) {

                        LOG_MESSAGE("Sync pulse deteceted.\n");

                        // start a new sequence after the successfull sync
                        syncDetected = true;
                        ClearReceivedBits();
                    }
                }
            }
        }	

        // remeber the current rising edge and clean the previous falling edge
        previousRisingEdge = true;
        previousFallingEdge = false;
        previousRisingEdgeTime = time;
        previousFallingEdgeTime = 0;
    
    } else {

        LOG_MESSAGE("Handling falling edge.\n");

        // keep the falling edge time
        previousFallingEdge = true;
        previousFallingEdgeTime = time;
    }
}

void RC433HQBasicSyncPulseDecoder::ClearReceivedBits()
{
    memset(receivedData, 0, RC433HQ_MAX_PULSE_BITS >> 3);
    receivedBits = 0;
}

void RC433HQBasicSyncPulseDecoder::StoreReceivedBit(byte bit)
{
    if (receivedBits < RC433HQ_MAX_PULSE_BITS) {
    
        // store the bit
        size_t offset = (receivedBits >> 3);
        receivedData[offset] = (receivedData[offset] << 1) | bit;

        // increase the number of stored bits
        receivedBits++;
    }
}