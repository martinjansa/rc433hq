#line 2 "rc433hq_tests.ino"
#include <string.h>
#include <ArduinoUnit.h>

#if defined(ARDUINO)
#	include <rc433hq.h>
#else
#	include "../../rc433hq.h"
#endif // defined(ARDUINO)

class Logger: public IRC433Logger {
public:
	virtual void LogMessage(const char *message)
  {
    printf(message);
  }
};


class TestingPulseGenerator {
private:
  IRC433PulseDecoder &decoder;
  RC433HQMilliseconds lastPulseStart;
public:
  TestingPulseGenerator(IRC433PulseDecoder &adecoder):
    decoder(adecoder),
    lastPulseStart(0)
  {
  }

  void SendEdge(bool high, RC433HQMilliseconds delayAfter)
  {
    decoder.HandleEdge(lastPulseStart, high);
    lastPulseStart += delayAfter;
  }

  void GeneratePulse(RC433HQMilliseconds highDuration, RC433HQMilliseconds lowDuration)
  {
    SendEdge(true, highDuration);
    SendEdge(false, lowDuration);
  }

  void GeneratePulses(RC433HQMilliseconds highDuration, RC433HQMilliseconds lowDuration, size_t count)
  {
    for (size_t i = 0; i < count; i++) {
      GeneratePulse(highDuration, lowDuration);
    }
  }
};

class PulseDecoderMock: public IRC433PulseDecoder {
private:
  RC433HQMilliseconds times[8];
  bool edges[8];
  size_t pos;
public:
	PulseDecoderMock(): pos(0) {}
	virtual void HandleEdge(RC433HQMilliseconds time, bool direction)
  {
    // is there is space in the iternal buffers
    if (pos < 8) {

      // store the incoming edge into the internal buffers
      times[pos] = time;
      edges[pos] = direction;
      pos++;
    }
  }

  void AssertHandleEdgeCalled(RC433HQMilliseconds expectedTimes[], bool expectedEdges[], size_t expectedEdgesCount)
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


test(NoiseFilterShouldForwardSlowPulse)
{  
  // given
  PulseDecoderMock mock;
  NoiseFilter filter(mock, 3);
  TestingPulseGenerator generator(filter);

  // when
  generator.GeneratePulses(10, 10, 2);

  // then
  RC433HQMilliseconds expectedTimes[] = { 0, 10, 20 };
  bool expectedEdges[] = { true, false, true};
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 3);
}

test(NoiseFilterShouldElimitateNoiseBeforeDownEdge)
{  
  // given
  PulseDecoderMock mock;
  NoiseFilter filter(mock, 3);
  TestingPulseGenerator generator(filter);

  // when
  generator.SendEdge(true, 9);
  generator.SendEdge(false, 0); // noise 
  generator.SendEdge(true, 1);  // noise 
  generator.SendEdge(false, 10);
  generator.GeneratePulse(10, 10);

  // then
  RC433HQMilliseconds expectedTimes[] = { 0, 10, 20 };
  bool expectedEdges[] = { true, false, true};
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 3);
}

test(NoiseFilterShouldIgnoreTheSameDirectionEdge)
{  
  // given
  PulseDecoderMock mock;
  NoiseFilter filter(mock, 3);
  TestingPulseGenerator generator(filter);

  // when
  generator.SendEdge(true, 5);
  generator.SendEdge(false, 5); // noise 
  generator.SendEdge(false, 10);
  generator.GeneratePulse(10, 10);

  // then
  RC433HQMilliseconds expectedTimes[] = { 0, 10, 20 };
  bool expectedEdges[] = { true, false, true};
  mock.AssertHandleEdgeCalled(expectedTimes, expectedEdges, 3);
}

test(BasicPulseDecoderShouldDecodeExactOneBitFollowedBySyncForMaxLen1)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 1);
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

test(BasicPulseDecoderShouldDecodeExactZeroBitFollowedBySyncForMaxLen1)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 1);
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

test(BasicPulseDecoderShouldDecodeExactOneBitFollowedBySyncForMaxLen2)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 2);
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

test(BasicPulseDecoderShouldDecodeExactZeroBitFollowedBySyncForMaxLen2)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 2);
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

test(BasicPulseDecoderShouldDecodeExactOneBitFollowedByInvalidSignalForMaxLen1)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 1);
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

test(BasicPulseDecoderShouldDecodeExactOneBitFollowedByInvalidSignalForMaxLen2)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 2);
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

test(BasicPulseDecoderShouldDecode16ExactOneBitsFollowedByInvalidSignalForMaxLen24)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 24);
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

test(BasicPulseDecoderShouldDecode16ExactZeroBitsFollowedByInvalidSignalForMaxLen24)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 24);
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

test(BasicPulseDecoderShouldInoreExact2OneBitsFollowedByInvalidSignalForMinLen3)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 3, 3);
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

