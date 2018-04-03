#include "rc433hq.h"

#include <string.h>
#include <stdio.h>

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
    missedCount(0),
    lastSentEdgeTime(0)
{
    buffer = new BufferValue[bufferSize];
}

RC433HQPulseBuffer::~RC433HQPulseBuffer()
{
    delete [] buffer; buffer = 0;
}

void RC433HQPulseBuffer::ProcessData(size_t &reportedBufferUsedCount, size_t &reportedProcessedCount, size_t &reportedMissedCount)
{
    // initialize the output values
    reportedBufferUsedCount = 0;
    reportedProcessedCount = 0;
    reportedMissedCount = 0;

    bool continueProcessing = false;
	
    bool missedIndexSet = false;
    size_t missedIndex = 0;;

    do {

        continueProcessing = false;

        bool dataAvailable = false;
        RC433HQMicroseconds time;
        bool direction;

        bool sendHandleMissedEdges = false;

        // disable the interrupts for a while
        noInterrupts();
        
        // if there are some items in the buffer
        if (usedCount > 0) {

            // set the data available flag
            dataAvailable = true;

            // read the first item from the buffer
            BufferValue value = buffer[dataIndex]; dataIndex = CalculateNext(dataIndex); usedCount--; reportedBufferUsedCount++;

            // calculate the direction
            direction = (value & 0x8000) != 0;

            // if the value is a marker that the absolute time is stored
            if ((value & 0x7fff) == 0x7fff) {

                // there should be two more values
                // assert(usedCount >= 2);

                // read the absolute time from the buffer
                RC433HQMicroseconds timeLow = buffer[dataIndex]; dataIndex = CalculateNext(dataIndex); usedCount--; reportedBufferUsedCount++;
                RC433HQMicroseconds timeHigh = buffer[dataIndex]; dataIndex = CalculateNext(dataIndex); usedCount--; reportedBufferUsedCount++;

                // calculate the time from two words
                time = (timeHigh << 16) | timeLow;

            } else {

                // we will calculate the relative time
                time = lastSentEdgeTime + (value & 0x7fff);
            }

            // if the missed index has been set and points to the next
            if (missedIndexSet && (missedIndex == dataIndex)) {

                // set the flag to call the handle missed edges
                sendHandleMissedEdges = true;

                // clear the value of the free index
                missedIndexSet = false;
                missedIndex = freeIndex;
            }

            // if there are still some data
            if (usedCount > 0) {

                // set the flag to make one more iteration
                continueProcessing = true;
            }
        }

        // if there were some missed items
        if (missedCount > 0) {
            
            // if the missed index has not been set
            if (!missedIndexSet) {

                // keep the current value of the free index
                missedIndexSet = true;
                missedIndex = freeIndex;
            }

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
            lastSentEdgeTime = time;

            // increase the reported user count
            reportedProcessedCount++;
        }

        // if we should send the handle missed edges
        if (sendHandleMissedEdges) {

            // call it
            connectedPulseDecoder.HandleMissedEdges();
            sendHandleMissedEdges = false;
        }

    } while (continueProcessing);
}

void RC433HQPulseBuffer::HandleEdge(RC433HQMicroseconds time, bool direction)
{
    // this method is typically called from an interrupt, so we use a simple mutual exclution 
    // just by disabling the interrupts from the method that reads the data written here.

    // assert(buffer != 0);

    bool successfullyStored = false;

    // if the buffer is valid
    if (buffer != 0) {

        // calculate the direction mask
        word directionMask = (direction? 0x8000: 0x0000);

        // if there are no items in the buffer
        if (usedCount == 0) {

            // store the absolut time of the edge
            StoreAbsolutTime(time, directionMask);
            lastStoredEdgeTime = time;
            successfullyStored = true;

        } else {

            // calculate the relative edge time from the current and last edge times
            RC433HQMicroseconds relativeEdgeTime = time - lastStoredEdgeTime;

            // if the time is smaller than what fits into 15 bits
            if (relativeEdgeTime < 0x7fff) {

                // if there is one item available in the buffer
                if (usedCount < bufferSize) {

                    // store the relative edge time
                    buffer[freeIndex] = directionMask | BufferValue(relativeEdgeTime); freeIndex = CalculateNext(freeIndex);  // direction and 15 bits
                    usedCount++;
                    lastStoredEdgeTime = time;
                    successfullyStored = true;
                }

            } else {

                // we need to store the absolute time. If there are at least 3 items in the buffer
                if (usedCount <= (bufferSize - 3)) {

                    // store the absolut time of the edge
                    StoreAbsolutTime(time, directionMask);
                    lastStoredEdgeTime = time;
                    successfullyStored = true;
                }
            }
        }
    }

    // if we could not successfully store the incoming value
    if (!successfullyStored) {

        // increase the missedCount
        missedCount++;
    }
}

void RC433HQPulseBuffer::StoreAbsolutTime(RC433HQMicroseconds time, BufferValue directionMask)
{
    buffer[freeIndex] = directionMask | 0x7fff; freeIndex = CalculateNext(freeIndex);      // direction and marker
    buffer[freeIndex] = BufferValue(time & 0xffff); freeIndex = CalculateNext(freeIndex);  // lower 2 bytes
    buffer[freeIndex] = BufferValue((time >> 16) & 0xffff); freeIndex = CalculateNext(freeIndex);  // higher 2 bytes
    usedCount += 3;
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

void RC433HQNoiseFilter::HandleMissedEdges()
{
    // ignore the last pulse and also the one currently obtained
    lastEdgeValid = false;

    // inform the attached processor we are losing some data
    decoder.HandleMissedEdges();    
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

                // calculate the delta
                CalculateDelta(highDuration, oneFirstUs);
                CalculateDelta(lowDuration, oneSecondUs);

                // store the bit
                StoreReceivedBit(1);

                // if we reached the higher limit of the received bit
                if (receivedBits == maxBits) {

                    LOG_MESSAGE("Max bits received, sending data.\n");

                    // send the data to the data receiver and clear the buffer
                    SendReceivedData();
                    syncDetected = false;
                }

            } else {

                // if the last pulse represented a 1
                if (syncDetected && EqualWithTolerance(highDuration, zeroFirstUs, toleranceUs) && EqualWithTolerance(lowDuration, zeroSecondUs, toleranceUs)) {

                    LOG_MESSAGE("Sync detected and pulse represents bit 0.\n");

                    // calculate the delta
                    CalculateDelta(highDuration, zeroFirstUs);
                    CalculateDelta(lowDuration, zeroSecondUs);

                    // store the bit
                    StoreReceivedBit(0);

                    // if we reached the higher limit of the received bit
                    if (receivedBits == maxBits) {

                        LOG_MESSAGE("Max bits received, sending data.\n");

                        // send the data to the data receiver and clear the buffer
                        SendReceivedData();

                        // no sync anymore
                        syncDetected = false;
                    }

                } else {

                    // no data pulse was detected

                    // if some data were received before and is above the minimal length
                    if (receivedBits >= minBits) {

                        LOG_MESSAGE("Min bits received before non data pulse, sending data.\n");

                        // send the data to the data receiver and clear the buffer
                        SendReceivedData();

                        // we are out of sync now
                        syncDetected = false;
                    }

                    // if the sync pulse was detected
                    if (EqualWithTolerance(highDuration, syncFirstUs, toleranceUs) && EqualWithTolerance(lowDuration, syncSecondUs, toleranceUs)) {

                        LOG_MESSAGE("Sync pulse deteceted.\n");

                        // calculate the delta of the sync
                        ClearDelta();
                        CalculateDelta(highDuration, syncFirstUs);
                        CalculateDelta(lowDuration, syncSecondUs);

                        // start a new sequence after the successfull sync
                        ClearReceivedBits();
                        syncDetected = true;
                        syncTime = previousRisingEdgeTime;
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

void RC433HQBasicSyncPulseDecoder::HandleMissedEdges()
{
    LOG_MESSAGE("Handling missed edges call.\n");

    // ignore the currently cached data, we will start over from the looking for the next sync
    ClearDelta();
    ClearReceivedBits();
    syncDetected = true;
}

void RC433HQBasicSyncPulseDecoder::CalculateDelta(RC433HQMicroseconds expected, RC433HQMicroseconds actual)
{
    if (toleranceUs != 0) {
        double delta;
        if (expected < actual) {
            delta = actual - expected;
        } else {
            delta = expected - actual;
        }
        deltaPowerSum += (delta * delta);
    }
}

void RC433HQBasicSyncPulseDecoder::ClearDelta()
{
    LOG_MESSAGE("Clearing delta sum.\n");
    deltaPowerSum = 0;
}

void RC433HQBasicSyncPulseDecoder::StoreReceivedBit(byte bit)
{
    if (receivedBits < RC433HQ_MAX_PULSE_BITS) {
    
        LOG_MESSAGE("Storing bit.\n");

        // store the bit
        size_t offset = (receivedBits >> 3);
        receivedData[offset] = (receivedData[offset] << 1) | bit;

        // increase the number of stored bits
        receivedBits++;
    }
}

void RC433HQBasicSyncPulseDecoder::ClearReceivedBits()
{
    LOG_MESSAGE("Clearing recevied bits cache.\n");

    memset(receivedData, 0, RC433HQ_MAX_PULSE_BITS >> 3);
    receivedBits = 0;
}

void RC433HQBasicSyncPulseDecoder::SendReceivedData()
{
    // data quality is 100% - totalDelta / tolerance. The range is 0% (always at tolerance) to 100% (always exact)
    double quality;
    
    if (toleranceUs != 0) {

        // calcutate the precission out of the average of the squares of the deltas for all edges
        double totalDelta = sqrt(deltaPowerSum / (2 *  (receivedBits + 1)));
        quality = 100.0 * (1.0 - (totalDelta / toleranceUs));

#       if defined DEBUG && !defined ARDUINO

        char buf[128];
        sprintf(buf, "Calculating quality with tolerace = %i, delta sq sum = %f, quality = %f.\n", int(toleranceUs), deltaPowerSum, quality);
        LOG_MESSAGE(buf);

#       endif // defined DEBUG

    } else {

        LOG_MESSAGE("Setting quality as 100 per cent.\n");

        quality = 100.0;
    }

    // send the data to the data receiver
    dataReceiver.HandleData(syncTime, receivedData, receivedBits, quality);
    ClearReceivedBits();
}

//////////////////////////////////////////////////////////////////////////////////
// RC433HQBasicSyncPulseEncoder implementation
//////////////////////////////////////////////////////////////////////////////////

void RC433HQBasicSyncPulseEncoder::EncodeData(RC433HQDataTransmitterBase &transmitter, const byte *data, size_t bits, size_t repetitions)
{
    for (size_t r = 0; r < repetitions; r++) {

        // encode sync pulse
        transmitter.TransmitPulse(syncFirstUs, syncSecondUs);

        size_t bytes = ((bits + 7) >> 3);

        // for all the bytes
        for (size_t i = 0; i < bytes; i++) {

            byte byteValue = data[i];

            size_t bitsInThisByte = 8;

            // if this is the last byte
            if ((i + 1 == bytes) && ((bits & 0x7) != 0)) {

                bitsInThisByte = (bits & 0x7);

                // remove unused higher bits from the last byte value
                for (size_t j = 8; j > bitsInThisByte; j--) {
                    byteValue = byteValue << 1;
                }
            }

            for (size_t j = 0; j < bitsInThisByte; j++) {

                // get the highest bit
                bool bitValue = ((byteValue & 0x80) != 0);
                byteValue = byteValue << 1;

                // if the bit is 1
                if (bitValue) {

                    // encode 1 pulse
                    transmitter.TransmitPulse(oneFirstUs, oneSecondUs);

                } else {

                    // encode 0 pulse
                    transmitter.TransmitPulse(zeroFirstUs, zeroSecondUs);
                }
            }
        }
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

//////////////////////////////////////////////////////////////////////////////////
// RC433HQReceiver implementation
//////////////////////////////////////////////////////////////////////////////////

RC433HQTransmitter::RC433HQTransmitter(int atransmitterGpioPin):
    transmitterGpioPin(atransmitterGpioPin),
    inTransitionMode(false),
    maxDelayTolerance(0),
    totalDelayedEdgesDelayTime(0),
    transmissionQualityStatistics(0),
    durationFinishTimeValid(false)
{
    // initialize the GPIO pin for output
    pinMode(transmitterGpioPin, OUTPUT);
}

RC433HQTransmitter::~RC433HQTransmitter()
{
}

// initializes the data transmission. Data can be sent only if the transmission is started
void RC433HQTransmitter::StartTransmission(RC433HQMicroseconds quietPeriodDurationBefore, RC433HQMicroseconds amaxDelayTolerance, RC433HQTransmissionQualityStatistics *atransmissionQualityStatistics)
{
    // assert(inTransitionMode == false);

    // initialize the transmission
    inTransitionMode = true;
    maxDelayTolerance = amaxDelayTolerance;
    totalDelayedEdgesDelayTime = 0;
    transmissionQualityStatistics = atransmissionQualityStatistics;
    durationFinishTimeValid = false;

    // initialize the statistics
    if (transmissionQualityStatistics) {

        transmissionQualityStatistics->countOfTransmittedEdges = 0;
        transmissionQualityStatistics->countOfDelayedEdges = 0;
        transmissionQualityStatistics->countOfDelayedEdgesOutsideOfTolerance = 0;
        transmissionQualityStatistics->averageDelay = 0;
    }
    
    // if there should be quiet period before the transmission
    if (quietPeriodDurationBefore > 0) {

        // plan the wait for the end of the quiet period
        PlanWaitForDuration(quietPeriodDurationBefore);
    }

}

// finalizes the data transmission
void RC433HQTransmitter::EndTransmission(RC433HQMicroseconds quietPeriodDurationAfter)
{
    // assert(inTransitionMode == true);

    // finish the calculation of the statistics
    if (transmissionQualityStatistics) {

        // TODO: transmissionQualityStatistics
        transmissionQualityStatistics->averageDelay = double(totalDelayedEdgesDelayTime) / transmissionQualityStatistics->countOfDelayedEdges;
    }

    // wait for the previous duration to finish
    WaitForThePreviousDurationToFinish();

    // if the quiet period after the transmission is defined
    if (quietPeriodDurationAfter > 0) {

        // wait for the end of the quiet
        PlanWaitForDuration(quietPeriodDurationAfter);
        WaitForThePreviousDurationToFinish();
    }

    // clear the statistics pointer
    transmissionQualityStatistics = 0;

    // clear the transition flag
    inTransitionMode = false;
}

// transmit the rising or falling edge and plan waiting after it
void RC433HQTransmitter::TransmitEdge(bool direction, RC433HQMicroseconds duration)
{
    // wait for the previous duration to finish
    WaitForThePreviousDurationToFinish();

    // write the value to the pin
    digitalWrite(transmitterGpioPin, (direction? HIGH: LOW));

    // plan the wait for the end of the pulse duration
    PlanWaitForDuration(duration);
}

// report the delay of the transition of the edge into the statistics
void RC433HQTransmitter::ReportEdgeTransitionDelay(RC433HQMicroseconds delay)
{
    // if the statistics structure is provided
    if (transmissionQualityStatistics) {

        // increment the total count
        transmissionQualityStatistics->countOfTransmittedEdges++;

        // if the edge was delayed
        if (delay > 0) {

            // incement the counter of delayed edges
            transmissionQualityStatistics->countOfDelayedEdges++;

            // add the total delay for later calculation
            totalDelayedEdgesDelayTime += delay;

            // if the delay was bigger than tolerance
            if (delay > maxDelayTolerance) {

                // incement the counter of delayed edges outside of tolerance
                transmissionQualityStatistics->countOfDelayedEdgesOutsideOfTolerance++;
            }
        }
    }
}

// plan wait for the duration
void RC433HQTransmitter::PlanWaitForDuration(RC433HQMicroseconds duration)
{
    // assert(inTransitionMode == true);

    // if the previous duration finish time is not valid
    if (!durationFinishTimeValid) {

        // initialize with the current time
        durationFinishTime = micros();

        // valid since now
        durationFinishTimeValid = true;
    }
        
    // calculate the end of the duration as relative to the previous finish time duration
    durationFinishTime += duration;
}

// if there was a delay after the previous edge or quiet period, wait for it to finish
void RC433HQTransmitter::WaitForThePreviousDurationToFinish()
{
    // initialize the delay value as there is no delay
    RC433HQMicroseconds delay = 0;

    // if we are waiting for the duration after the previous edge to finish
    if (durationFinishTimeValid) {

        // TODO: durationFinishTime

        // get the current time in us
        RC433HQMicroseconds now = micros();

        // calculate time left till end of the duration
        RC433HQMicroseconds durationLeftTime = durationFinishTime - now;

        // if the time difference is smaller than 1 minute, then we have not yet passed the target time and the subtraction did not overflow
        if (durationLeftTime < RC433HQMicroseconds(60000000)) {

            // if there is still some time to wait
            if (durationLeftTime > 0) {

                // wait 
                delayMicroseconds(durationLeftTime);

                // get the current time again (just for case the delay was not exact and we need to report delay in edge)
                now = micros();
            }
        }

        // calculate how much we are delayed
        delay = now - durationFinishTime;
    }

    // report the delay to the statistics
    ReportEdgeTransitionDelay(delay);
}

