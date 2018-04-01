#include "rc433hq.h"

#include <string.h>

bool EqualWithTolerance(RC433HQMicroseconds a, RC433HQMicroseconds b, RC433HQMicroseconds tolerance)
{
    return ((a - tolerance) <= b) && (b <= (a + tolerance));
}

//////////////////////////////////////////////////////////////////////////////////
// RC433HQPulseBuffer implementation
//////////////////////////////////////////////////////////////////////////////////

RC433HQPulseBuffer::RC433HQPulseBuffer(IRC433PulseProcessor &aconnectedPulseDecoder, size_t abufferSize):
    connectedPulseDecoder(aconnectedPulseDecoder),
    logger(0),
    bufferSize(abufferSize),
    dataIndex(0),
    freeIndex(0),
    usedCount(0),
    missedCount(0)
{
    times = new RC433HQMicroseconds[bufferSize];
    directions = new bool[bufferSize];
}

RC433HQPulseBuffer::~RC433HQPulseBuffer()
{
    delete [] times; times = 0;
    delete [] directions; directions = 0;
}

void RC433HQPulseBuffer::ProcessData(size_t &reportedUsedCount, size_t &reportedMissedCount)
{
    // initialize the output values
    reportedUsedCount = 0;
    reportedMissedCount = 0;

    bool continueProcessing = false;

    do {

        continueProcessing = false;

        bool dataAvailable = false;
        RC433HQMicroseconds time;
        bool direction;

        // disable the interrupts for a while
        noInterrupts();
        
        // if there are some items in the buffer
        if (usedCount > 0) {

            // set the data available flag
            dataAvailable = true;

            // keep the time and direction of the first edge in the buffer
            time = times[dataIndex];
            direction = directions[dataIndex];

            // move the data index further and decrease the used count
            dataIndex = CalculateNext(dataIndex);
            usedCount--;

            // if there are still some data
            if (usedCount > 0) {

                // set the flag to make one more iteration
                continueProcessing = true;
            }
        }

        // if there were some missed items
        if (missedCount > 0) {
            
            // keep the number of missed items
            reportedMissedCount += missedCount;

            // clear the original missed items counter
            missedCount = 0;
        }

        // re-enable interrupts
        interrupts();

        // if some data was removed from the buffer
        if (dataAvailable) {

            // send the data to the connected decoder
            connectedPulseDecoder.HandleEdge(time, direction);

            // increase the reported user count
            reportedUsedCount++;
        }

    } while (continueProcessing);
}

void RC433HQPulseBuffer::HandleEdge(RC433HQMicroseconds time, bool direction)
{
    // this method is typically called from an interrupt, so we use a simple mutual exclution 
    // just by disabling the interrupts from the method that reads the data written here.

    // if there is space in the bufferSize
    if (times != 0 && directions != 0 && usedCount < bufferSize) {

        // store the value into the buffer
        times[freeIndex] = time;
        directions[freeIndex] = direction;

        // move the free index and increase the used count
        freeIndex = CalculateNext(freeIndex);
        usedCount++;

    } else {

        // there is no space in the buffer. Increase the missedCount
        missedCount++;
    }
}

size_t RC433HQPulseBuffer::CalculateNext(size_t index)
{
    if (index < bufferSize) {
        return index + 1;
    } else {
        return 0;
    }
}

//////////////////////////////////////////////////////////////////////////////////
// RC433HQNoiseFilter implementation
//////////////////////////////////////////////////////////////////////////////////

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


//////////////////////////////////////////////////////////////////////////////////
// RC433HQBasicSyncPulseDecoder implementation
//////////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////////
// RC433HQReceiver implementation
//////////////////////////////////////////////////////////////////////////////////

// initialization of the static member
RC433HQReceiver *RC433HQReceiver::activeInstance = 0;

RC433HQReceiver::RC433HQReceiver(IRC433PulseProcessor &adecoder, int areceiverGpioPin):
    decoder(adecoder),
    iAmActiveInstance(false),
	receiverGpioPin(areceiverGpioPin)
{
    if (activeInstance == 0) {
        activeInstance = this;
        iAmActiveInstance = true;
        pinMode(receiverGpioPin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(receiverGpioPin), RC433HQReceiver::HandleInterrupt, CHANGE);
    }
}

RC433HQReceiver::~RC433HQReceiver()
{
    if (iAmActiveInstance) {
        detachInterrupt(digitalPinToInterrupt(receiverGpioPin));
        activeInstance = 0;
    }
}

void RC433HQReceiver::HandleInterrupt()
{
    // if the active instance is defined
    // assert(activeInstance != 0);

    activeInstance->HandleInterruptInternal();
}

void RC433HQReceiver::HandleInterruptInternal()
{
    // get the current time
    RC433HQMicroseconds now = micros();

    // get the status of the pin
    int pinState = digitalRead(receiverGpioPin);

    // convert into boolen (true = rising, false = falling edge)
    bool direction = (pinState == HIGH);

    // call the decoder to handle the receiver edge change
    decoder.HandleEdge(now, direction);
}
