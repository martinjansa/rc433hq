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

        // send the last edge into the connected decoder
        decoder.HandleEdge(lastEdgeTime, lastEdgeDirection);
    }

    // keep the currect edge in the memory
    lastEdgeTime = time;
    lastEdgeDirection = direction;
    lastEdgeValid = true;
}