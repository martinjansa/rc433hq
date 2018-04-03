#line 2 "rc433hq_tests.ino"
#include <string.h>
#include <ArduinoUnit.h>

#if defined(ARDUINO)
#	include <rc433hq.h>
#else
#	include "../../rc433hq.h"
#endif // defined(ARDUINO)

//////////////////////////////////////////////////////////////////////////////////
// Utility and mock classes
//////////////////////////////////////////////////////////////////////////////////

class Logger: public IRC433Logger {
public:
	virtual void LogMessage(const char *message)
  {
    printf(message);
  }
};


class TestingPulseGenerator {
private:
  IRC433PulseProcessor &decoder;
  RC433HQMicroseconds lastPulseStart;
public:
  TestingPulseGenerator(IRC433PulseProcessor &adecoder):
    decoder(adecoder),
    lastPulseStart(0)
  {
  }

  void SendEdge(bool direction, RC433HQMicroseconds delayAfter)
  {
    decoder.HandleEdge(lastPulseStart, direction);
    lastPulseStart += delayAfter;
  }

  void GeneratePulse(RC433HQMicroseconds highDuration, RC433HQMicroseconds lowDuration)
  {
    SendEdge(true, highDuration);
    SendEdge(false, lowDuration);
  }

  void GeneratePulses(RC433HQMicroseconds highDuration, RC433HQMicroseconds lowDuration, size_t count)
  {
    for (size_t i = 0; i < count; i++) {
      GeneratePulse(highDuration, lowDuration);
    }
  }
};

class PulseDecoderMock: public IRC433PulseProcessor {
private:
  RC433HQMicroseconds times[8];
  bool edges[8];
  size_t pos;
  bool handleMissedEdgesCalled;
public:
	PulseDecoderMock():
    pos(0),
    handleMissedEdgesCalled(false)
  {
  }

	virtual void HandleEdge(RC433HQMicroseconds time, bool direction)
  {
    // is there is space in the iternal buffers
    if (pos < 8) {

      // store the incoming edge into the internal buffers
      times[pos] = time;
      edges[pos] = direction;
      pos++;
    }
  }

  virtual void HandleMissedEdges()
  {
    handleMissedEdgesCalled = true;
  }

  void AssertHandleEdgeCalled(RC433HQMicroseconds expectedTimes[], bool expectedEdges[], size_t expectedEdgesCount)
  {
    assertEqual(pos, expectedEdgesCount);
    for (size_t i = 0; i < pos; i++) {
      assertEqual(times[i], expectedTimes[i]);
      assertEqual(edges[i], expectedEdges[i]);
    }
  }

  void AssertHandleMissedEdgesNotCalled()
  {
    assertEqual(handleMissedEdgesCalled, false);
  }

  void AssertHandleMissedEdgesCalled()
  {
    assertEqual(handleMissedEdgesCalled, true);
  }
};

class DataReceiverMock: public IRC433DataReceiver {
private:
  RC433HQMicroseconds storedTime;
  byte storedData[16];
  size_t storedBitsCount;
  double storedQuality;
public:
  DataReceiverMock():
    storedBitsCount(0),
    storedQuality(0.0)
  {
  }
  virtual void HandleData(RC433HQMicroseconds time, const byte *data, size_t bits, double quality)
  {
    storedTime = time;

    // if the is enough space in the internal buffer
    if ((0 < bits) && (bits <= (8 * 16))) {

      // store the data into the internal
      memcpy(storedData, data, (bits + 7) >> 3);
      storedBitsCount = bits;

    } else {

      storedBitsCount = 0;
    }
    storedQuality = quality;
  }
  
  void AssertHandleDataCalled(const byte *expectedData, size_t expectedBits, double expectedQualityMin = 0.0, double expectedQualityMax = 100.0)
  {
    assertEqual(storedBitsCount, expectedBits);
    for (size_t i = 0; i < (storedBitsCount >> 3); i++) {
      assertEqual(storedData[i], expectedData[i]);
    }
    ASSERT_LE_3(expectedQualityMin, storedQuality, "stored quality above the minimal value");
    ASSERT_LE_3(storedQuality, expectedQualityMax, "stored quality bellow the maximal value");
  }
};

class TransmitterMock: public RC433HQDataTransmitterBase {
private:
  size_t bufferCapacity;
  size_t bufferSize;
  bool *directions;
  RC433HQMicroseconds *durations;

public:
  TransmitterMock(size_t abufferCapacity):
    bufferCapacity(abufferCapacity),
    bufferSize(0)
  {
    directions = new bool [bufferCapacity];
    durations = new RC433HQMicroseconds [bufferCapacity];
  }

  ~TransmitterMock()
  {
    delete [] durations; durations = 0;
    delete [] directions; directions = 0;
  }

	virtual void TransmitEdge(bool direction, RC433HQMicroseconds duration)
  {
    // if there is space in the buffers
    if (directions && durations && bufferSize < bufferCapacity) {
      directions[bufferSize] = direction;
      durations[bufferSize] = duration;
    }
    bufferSize++;
  }

  void AssertEdgesTransmitted(const bool *expectedDirections, const RC433HQMicroseconds *expectedDurations, size_t expectedCount)
  {
    assertEqual(bufferSize, expectedCount);
    for (size_t i = 0; i < bufferSize; i++) {
      assertEqual(directions[i], expectedDirections[i]);
      assertEqual(durations[i], expectedDurations[i]);
    }
  }
};

//////////////////////////////////////////////////////////////////////////////////
// RC433HQPulseBuffer tests
//////////////////////////////////////////////////////////////////////////////////

test(RC433HQPulseBuffer_ShouldRemember1EdgeInSize1Buffer)
{  
  // given
  PulseDecoderMock mock;
  RC433HQPulseBuffer buffer(mock, 3);
  TestingPulseGenerator generator(buffer);

  // when
  generator.SendEdge(true, 9);
  size_t reportedBufferUsedCount = 0;
  size_t reportedProcessedCount = 0;
  size_t reportedMissedCount = 0;
  buffer.ProcessData(reportedBufferUsedCount, reportedProcessedCount, reportedMissedCount);

  // then
  RC433HQMicroseconds expectedTimes[] = { 0 };
  bool expectedEdges[] = { true };
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 1);
  assertEqual(reportedProcessedCount, 1);
  assertEqual(reportedMissedCount, 0);
  mock.AssertHandleMissedEdgesNotCalled();
}

test(RC433HQPulseBuffer_ShouldRemember4EdgesInSize6Buffer)
{  
  // given
  PulseDecoderMock mock;
  RC433HQPulseBuffer buffer(mock, 6);
  TestingPulseGenerator generator(buffer);

  // when
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 9);
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 9);
  size_t reportedBufferUsedCount = 0;
  size_t reportedProcessedCount = 0;
  size_t reportedMissedCount = 0;
  buffer.ProcessData(reportedBufferUsedCount, reportedProcessedCount, reportedMissedCount);

  // then
  RC433HQMicroseconds expectedTimes[] = { 0, 9, 18, 27 };
  bool expectedEdges[] = { true, false, true, false };
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 4);
  assertEqual(reportedProcessedCount, 4);
  assertEqual(reportedMissedCount, 0);
  mock.AssertHandleMissedEdgesNotCalled();
}

test(RC433HQPulseBuffer_ShouldForget4EdgesInSize6Buffer)
{  
  // given
  PulseDecoderMock mock;
  RC433HQPulseBuffer buffer(mock, 6);
  TestingPulseGenerator generator(buffer);

  // when
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 9);
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 9);
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 9);
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 9);
  size_t reportedBufferUsedCount = 0;
  size_t reportedProcessedCount = 0;
  size_t reportedMissedCount = 0;
  buffer.ProcessData(reportedBufferUsedCount, reportedProcessedCount, reportedMissedCount);

  // then
  RC433HQMicroseconds expectedTimes[] = { 0, 9, 18, 27 };
  bool expectedEdges[] = { true, false, true, false };
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 4);
  assertEqual(reportedProcessedCount, 4);
  assertEqual(reportedMissedCount, 4);
  mock.AssertHandleMissedEdgesCalled();
}

//////////////////////////////////////////////////////////////////////////////////
// RC433HQNoiseFilter tests
//////////////////////////////////////////////////////////////////////////////////

test(NoiseFilter_ShouldForwardSlowPulse)
{  
  // given
  PulseDecoderMock mock;
  RC433HQNoiseFilter filter(mock, 3);
  TestingPulseGenerator generator(filter);

  // when
  generator.GeneratePulses(10, 10, 2);

  // then
  RC433HQMicroseconds expectedTimes[] = { 0, 10, 20 };
  bool expectedEdges[] = { true, false, true};
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 3);
}

test(NoiseFilter_ShouldElimitateNoiseBeforeDownEdge)
{  
  // given
  PulseDecoderMock mock;
  RC433HQNoiseFilter filter(mock, 3);
  TestingPulseGenerator generator(filter);

  // when
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 0); // noise 
  generator.SendEdge(true, 1);  // noise 
  generator.SendEdge(false, 10);
  generator.GeneratePulse(10, 10);

  // then
  RC433HQMicroseconds expectedTimes[] = { 0, 10, 20 };
  bool expectedEdges[] = { true, false, true};
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 3);
}

test(NoiseFilter_ShouldIgnoreTheSameDirectionEdge)
{  
  // given
  PulseDecoderMock mock;
  RC433HQNoiseFilter filter(mock, 3);
  TestingPulseGenerator generator(filter);

  // when
  generator.SendEdge(true, 5);
  generator.SendEdge(false, 5); // noise 
  generator.SendEdge(false, 10);
  generator.GeneratePulse(10, 10);

  // then
  RC433HQMicroseconds expectedTimes[] = { 0, 10, 20 };
  bool expectedEdges[] = { true, false, true};
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 3);
}

//////////////////////////////////////////////////////////////////////////////////
// RC433HQBasicSyncPulseDecoder tests
//////////////////////////////////////////////////////////////////////////////////

test(BasicPulseDecoder_ShouldDecodePreciseOneBitFollowedBySyncForMaxLen1)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 1);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(40, 40);      // sync
  generator.GeneratePulse(30, 10);      // bit 1
  generator.GeneratePulse(40, 40);      // sync
  generator.SendEdge(true, 0);          // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0x01 };
  dataReceiverMock.AssertHandleDataCalled(expected, 1, 100.0, 100.0);
}

test(BasicPulseDecoder_ShouldDecodePreciseZeroBitFollowedBySyncForMaxLen1)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 1);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(40, 40);      // sync
  generator.GeneratePulse(10, 30);      // bit 0
  generator.GeneratePulse(40, 40);      // sync
  generator.SendEdge(true, 0);          // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0x00 };
  dataReceiverMock.AssertHandleDataCalled(expected, 1, 100.0, 100.0);
}

test(BasicPulseDecoder_ShouldDecodeUnpreciseLowOneBitFollowedBySyncForMaxLen1)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 1);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(42, 38);      // sync (unprecise)
  generator.GeneratePulse(28, 12);      // bit 1 (unprecise)
  generator.GeneratePulse(38, 42);      // sync (unprecise)
  generator.SendEdge(true, 0);          // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0x01 };
  dataReceiverMock.AssertHandleDataCalled(expected, 1, 0.0, 10.0);
}

test(BasicPulseDecoder_ShouldDecodeUnpreciseLowZeroBitFollowedBySyncForMaxLen1)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 1);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(39, 41);      // sync (unprecise)
  generator.GeneratePulse(9, 31);       // bit 0 (unprecise)
  generator.GeneratePulse(41, 39);      // sync (unprecise)
  generator.SendEdge(true, 0);          // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0x00 };
  dataReceiverMock.AssertHandleDataCalled(expected, 1, 45.0, 55.0);
}

test(BasicPulseDecoder_ShouldDecodePreciseOneBitFollowedBySyncForMaxLen2)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 2);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(40, 40);      // sync
  generator.GeneratePulse(30, 10);      // bit 1
  generator.GeneratePulse(40, 40);      // sync
  generator.SendEdge(true, 0);          // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0x01 };
  dataReceiverMock.AssertHandleDataCalled(expected, 1);
}

test(BasicPulseDecoder_ShouldDecodePreciseZeroBitFollowedBySyncForMaxLen2)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 2);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(40, 40);      // sync
  generator.GeneratePulse(10, 30);      // bit 0
  generator.GeneratePulse(40, 40);      // sync
  generator.SendEdge(true, 0);          // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0x00 };
  dataReceiverMock.AssertHandleDataCalled(expected, 1);
}

test(BasicPulseDecoder_ShouldDecodePreciseOneBitFollowedByInvalidSignalForMaxLen1)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 1);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(40, 40);      // sync
  generator.GeneratePulse(30, 10);      // bit 1
  generator.GeneratePulse(10, 10);      // invalid pulse
  generator.SendEdge(true, 0);          // last rising edge to allow detection of previous pulse

  // then
  byte expected[] = { 0x01 };
  dataReceiverMock.AssertHandleDataCalled(expected, 1);
}

test(BasicPulseDecoder_ShouldDecodePreciseOneBitFollowedByInvalidSignalForMaxLen2)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 2);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(40, 40);      // sync
  generator.GeneratePulse(30, 10);      // bit 1
  generator.GeneratePulse(10, 10);      // invalid pulse
  generator.SendEdge(true, 0);          // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0x01 };
  dataReceiverMock.AssertHandleDataCalled(expected, 1);
}

test(BasicPulseDecoder_ShouldDecode16PreciseOneBitsFollowedByInvalidSignalForMaxLen24)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 24);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(40, 40);       // sync
  generator.GeneratePulses(30, 10, 16);  // 16x bit 1
  generator.GeneratePulse(10, 10);       // invalid pulse
  generator.SendEdge(true, 0);           // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0xff, 0xff };
  dataReceiverMock.AssertHandleDataCalled(expected, 16);
}

test(BasicPulseDecoder_ShouldDecode16PreciseZeroBitsFollowedByInvalidSignalForMaxLen24)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 1, 24);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(40, 40);       // sync
  generator.GeneratePulses(10, 30, 16);  // 16x bit 1
  generator.GeneratePulse(10, 10);       // invalid pulse
  generator.SendEdge(true, 0);           // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0x00, 0x00 };
  dataReceiverMock.AssertHandleDataCalled(expected, 16);
}

test(BasicPulseDecoder_ShouldInorePrecise2OneBitsFollowedByInvalidSignalForMinLen3)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433HQBasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, 2, true, 3, 3);
  Logger logger;
  decoder.SetLogger(logger);
  TestingPulseGenerator generator(decoder);

  // when
  generator.GeneratePulse(40, 40);      // sync
  generator.GeneratePulses(10, 30, 2);  // 2x bit 1
  generator.GeneratePulse(10, 10);      // invalid pulse
  generator.SendEdge(true, 0);          // last rising edge to allow detection of previous pulse
  
  // then
  byte expected[] = { 0x00 };
  dataReceiverMock.AssertHandleDataCalled(expected, 0);
}


//////////////////////////////////////////////////////////////////////////////////
// TransmitterMock tests
//////////////////////////////////////////////////////////////////////////////////

test(BasicPulseEncoder_ShouldEncodeOneBit)
{
  // given
  TransmitterMock transmitterMock(4);
  RC433HQBasicSyncPulseEncoder encoder(40, 40, 10, 30, 30, 10, true);

  // when
  byte val = 0x01;
  encoder.EncodeData(transmitterMock, &val, 1, 1);
  
  // then
  const bool expectedDirections [] = { true, false, true, false };
  const RC433HQMicroseconds expectedDurations[] = { 40, 40, 30, 10 };
  transmitterMock.AssertEdgesTransmitted(expectedDirections, expectedDurations, 4);
}


test(BasicPulseEncoder_ShouldEncodeZeroBit)
{
  // given
  TransmitterMock transmitterMock(4);
  RC433HQBasicSyncPulseEncoder encoder(40, 40, 10, 30, 30, 10, true);

  // when
  byte val = 0x00;
  encoder.EncodeData(transmitterMock, &val, 1, 1);
  
  // then
  const bool expectedDirections [] = { true, false, true, false };
  const RC433HQMicroseconds expectedDurations[] = { 40, 40, 10, 30 };
  transmitterMock.AssertEdgesTransmitted(expectedDirections, expectedDurations, 4);
}


test(BasicPulseEncoder_ShouldEncode8Bits)
{
  // given
  TransmitterMock transmitterMock(18);
  RC433HQBasicSyncPulseEncoder encoder(40, 40, 10, 30, 30, 10, true);

  // when
  const byte val[] = { 0xAA };
  encoder.EncodeData(transmitterMock, val, 8, 1);
  
  // then
  const bool expectedDirections [] = { true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false };
  const RC433HQMicroseconds expectedDurations[] = { 40, 40, 30, 10, 10, 30, 30, 10, 10, 30, 30, 10, 10, 30, 30, 10, 10, 30 };
  transmitterMock.AssertEdgesTransmitted(expectedDirections, expectedDurations, 18);
}


test(BasicPulseEncoder_ShouldEncode16Bits)
{
  // given
  TransmitterMock transmitterMock(34);
  RC433HQBasicSyncPulseEncoder encoder(40, 40, 10, 30, 30, 10, true);

  // when
  const byte val[] = { 0xAA, 0x55 };
  encoder.EncodeData(transmitterMock, val, 16, 1);
  
  // then
  const bool expectedDirections [] = { true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false };
  const RC433HQMicroseconds expectedDurations[] = { 40, 40, 30, 10, 10, 30, 30, 10, 10, 30, 30, 10, 10, 30, 30, 10, 10, 30, 10, 30, 30, 10, 10, 30, 30, 10, 10, 30, 30, 10, 10, 30, 30, 10 };
  transmitterMock.AssertEdgesTransmitted(expectedDirections, expectedDurations, 34);
}


//////////////////////////////////////////////////////////////////////////////////
// Test driver infrastructure
//////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  while(!Serial) {} // Portability for Leonardo/Micro

  Test::min_verbosity |= TEST_VERBOSITY_ASSERTIONS_FAILED;
  //Test::min_verbosity |= TEST_VERBOSITY_ASSERTIONS_ALL;

  //Test::exclude("*");
  //Test::include("RC433HQPulseBuffer_ShouldForget4EdgesInSize4Buffer");
}

void loop()
{
  Test::run();
}

