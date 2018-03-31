#include "rc433hq.h"


/*
void RC433BasicSyncPulseDecoder::HandleEdge(unsigned long time, bool direction)
{
	
}
*/

void NoiseFilter::HandleEdge(unsigned long time, bool direction)
{
    // if we have some edge in the memory
	if (lastEdgeValid) {

        // calculate the duration of the last pulse
        unsigned long duration = (time - lastEdgeTime);

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