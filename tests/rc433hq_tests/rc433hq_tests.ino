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

  void SendEdge(bool high, RC433HQMicroseconds delayAfter)
  {
    decoder.HandleEdge(lastPulseStart, high);
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
public:
	PulseDecoderMock(): pos(0) {}
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

  void AssertHandleEdgeCalled(RC433HQMicroseconds expectedTimes[], bool expectedEdges[], size_t expectedEdgesCount)
  {
    assertEqual(pos, expectedEdgesCount);
    for (size_t i = 0; i < pos; i++) {
      assertEqual(times[i], expectedTimes[i]);
      assertEqual(edges[i], expectedEdges[i]);
    }
  }
};

class DataReceiverMock: public IRC433DataReceiver {
private:
  byte storedData[16];
  size_t storedBitsCount;
public:
  DataReceiverMock():
    storedBitsCount(0)
  {
  }
  virtual void HandleData(const byte *data, size_t bits)
  {
    // if the is enough space in the internal buffer
    if ((0 < bits) && (bits <= (8 * 16))) {

      // store the data into the internal
      memcpy(storedData, data, (bits + 7) >> 3);
      storedBitsCount = bits;

    } else {

      storedBitsCount = 0;
    }
  }
  
  void AssertHandleDataCalled(const byte *expectedData, size_t expectedBits)
  {
    assertEqual(storedBitsCount, expectedBits);
    for (size_t i = 0; i < (storedBitsCount >> 3); i++) {
      assertEqual(storedData[i], expectedData[i]);
    }
  }
};

//////////////////////////////////////////////////////////////////////////////////
// RC433HQNoiseFilter tests
//////////////////////////////////////////////////////////////////////////////////

test(RC433HQPulseBuffer_ShouldRemember1EdgeInSize1Buffer)
{  
  // given
  PulseDecoderMock mock;
  RC433HQPulseBuffer buffer(mock, 1);
  TestingPulseGenerator generator(buffer);

  // when
  generator.SendEdge(true, 9);
  size_t reportedUsedCount = 0;
  size_t reportedMissedCount = 0;
  buffer.ProcessData(reportedUsedCount, reportedMissedCount);

  // then
  RC433HQMicroseconds expectedTimes[] = { 0 };
  bool expectedEdges[] = { true };
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 1);
  assertEqual(reportedUsedCount, 1);
  assertEqual(reportedMissedCount, 0);
}

test(RC433HQPulseBuffer_ShouldRemember4EdgesInSize4Buffer)
{  
  // given
  PulseDecoderMock mock;
  RC433HQPulseBuffer buffer(mock, 4);
  TestingPulseGenerator generator(buffer);

  // when
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 9);
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 9);
  size_t reportedUsedCount = 0;
  size_t reportedMissedCount = 0;
  buffer.ProcessData(reportedUsedCount, reportedMissedCount);

  // then
  RC433HQMicroseconds expectedTimes[] = { 0, 9, 18, 27 };
  bool expectedEdges[] = { true, false, true, false };
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 4);
  assertEqual(reportedUsedCount, 4);
  assertEqual(reportedMissedCount, 0);
}

test(RC433HQPulseBuffer_ShouldForget4EdgesInSize4Buffer)
{  
  // given
  PulseDecoderMock mock;
  RC433HQPulseBuffer buffer(mock, 4);
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
  size_t reportedUsedCount = 0;
  size_t reportedMissedCount = 0;
  buffer.ProcessData(reportedUsedCount, reportedMissedCount);

  // then
  RC433HQMicroseconds expectedTimes[] = { 0, 9, 18, 27 };
  bool expectedEdges[] = { true, false, true, false };
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 4);
  assertEqual(reportedUsedCount, 4);
  assertEqual(reportedMissedCount, 4);
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
  dataReceiverMock.AssertHandleDataCalled(expected, 1);
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
  dataReceiverMock.AssertHandleDataCalled(expected, 1);
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
  dataReceiverMock.AssertHandleDataCalled(expected, 1);
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
  dataReceiverMock.AssertHandleDataCalled(expected, 1);
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
// Test driver infrastructure
//////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  while(!Serial) {} // Portability for Leonardo/Micro

  Test::min_verbosity |= TEST_VERBOSITY_ASSERTIONS_FAILED;
}

void loop()
{
  Test::run();
}

