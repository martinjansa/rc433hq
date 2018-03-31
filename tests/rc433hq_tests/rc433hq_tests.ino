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

  // when
  filter.HandleEdge(0, true);
  filter.HandleEdge(10, false);
  filter.HandleEdge(20, true);
  filter.HandleEdge(30, false);

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

  // when
  filter.HandleEdge(0, true);
  filter.HandleEdge(9, false); // noise 
  filter.HandleEdge(9, true);  // noise 
  filter.HandleEdge(10, false);
  filter.HandleEdge(20, true);
  filter.HandleEdge(30, false);

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

  // when
  filter.HandleEdge(0, true);
  filter.HandleEdge(5, false); // noise 
  filter.HandleEdge(10, false);
  filter.HandleEdge(20, true);
  filter.HandleEdge(30, false);

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

  // when
  decoder.HandleEdge(0, true);    // sync start
  decoder.HandleEdge(40, false);  // sync
  decoder.HandleEdge(80, true);   // 1st bit start
  decoder.HandleEdge(110, false); // value 1
  decoder.HandleEdge(120, true);  // sync start
  decoder.HandleEdge(160, false); // sync
  decoder.HandleEdge(200, true);  // end of sync
  
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

  // when
  decoder.HandleEdge(0, true);    // sync start
  decoder.HandleEdge(40, false);  // sync
  decoder.HandleEdge(80, true);   // 1st bit start
  decoder.HandleEdge(90, false); // value 0
  decoder.HandleEdge(120, true);  // sync start
  decoder.HandleEdge(160, false); // sync
  decoder.HandleEdge(200, true);  // end of sync
  
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

  // when
  decoder.HandleEdge(0, true);    // sync start
  decoder.HandleEdge(40, false);  // sync
  decoder.HandleEdge(80, true);   // 1st bit start
  decoder.HandleEdge(110, false); // value 1
  decoder.HandleEdge(120, true);  // sync start
  decoder.HandleEdge(160, false); // sync
  decoder.HandleEdge(200, true);  // end of sync
  
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

  // when
  decoder.HandleEdge(0, true);    // sync start
  decoder.HandleEdge(40, false);  // sync
  decoder.HandleEdge(80, true);   // 1st bit start
  decoder.HandleEdge(90, false); // value 0
  decoder.HandleEdge(120, true);  // sync start
  decoder.HandleEdge(160, false); // sync
  decoder.HandleEdge(200, true);  // end of sync
  
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

  // when
  decoder.HandleEdge(0, true);    // sync start
  decoder.HandleEdge(40, false);  // sync
  decoder.HandleEdge(80, true);   // 1st bit start
  decoder.HandleEdge(110, false); // value 1
  decoder.HandleEdge(120, true);  // invalid pulse start
  decoder.HandleEdge(110, false); // invalid pulse
  decoder.HandleEdge(120, true);  // end of invalid pulse
  
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

  // when
  decoder.HandleEdge(0, true);    // sync start
  decoder.HandleEdge(40, false);  // sync
  decoder.HandleEdge(80, true);   // 1st bit start
  decoder.HandleEdge(110, false); // value 1
  decoder.HandleEdge(120, true);  // invalid pulse start
  decoder.HandleEdge(110, false); // invalid pulse
  decoder.HandleEdge(120, true);  // end of invalid pulse
  
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

  // when
  decoder.HandleEdge(0, true);    // sync start
  decoder.HandleEdge(40, false);  // sync
  decoder.HandleEdge(80, true);   // 1st bit start
  decoder.HandleEdge(110, false); // value 1
  decoder.HandleEdge(120, true);  // 2nd bit start
  decoder.HandleEdge(150, false); // value 1
  decoder.HandleEdge(160, true);  // 3rd bit start
  decoder.HandleEdge(190, false); // value 1
  decoder.HandleEdge(200, true);  // 4th bit start
  decoder.HandleEdge(230, false); // value 1
  decoder.HandleEdge(240, true);  // 5th bit start
  decoder.HandleEdge(270, false); // value 1
  decoder.HandleEdge(280, true);  // 6th bit start
  decoder.HandleEdge(310, false); // value 1
  decoder.HandleEdge(320, true);  // 7th bit start
  decoder.HandleEdge(350, false); // value 1
  decoder.HandleEdge(360, true);  // 8th bit start
  decoder.HandleEdge(390, false); // value 1
  decoder.HandleEdge(400, true);  // 9th bit start
  decoder.HandleEdge(430, false); // value 1
  decoder.HandleEdge(440, true);  // 10th bit start
  decoder.HandleEdge(470, false); // value 1
  decoder.HandleEdge(480, true);  // 11th bit start
  decoder.HandleEdge(510, false); // value 1
  decoder.HandleEdge(520, true);  // 12th bit start
  decoder.HandleEdge(550, false); // value 1
  decoder.HandleEdge(560, true);  // 13th bit start
  decoder.HandleEdge(590, false); // value 1
  decoder.HandleEdge(600, true);  // 14th bit start
  decoder.HandleEdge(630, false); // value 1
  decoder.HandleEdge(640, true);  // 15th bit start
  decoder.HandleEdge(670, false); // value 1
  decoder.HandleEdge(680, true);  // 15th bit start
  decoder.HandleEdge(710, false); // value 1
  decoder.HandleEdge(720, true);  // invalid pulse start
  decoder.HandleEdge(730, false); // invalid pulse
  decoder.HandleEdge(740, true);  // end of invalid pulse
  
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

  // when
  decoder.HandleEdge(0, true);    // sync start
  decoder.HandleEdge(40, false);  // sync
  decoder.HandleEdge(80, true);   // 1st bit start
  decoder.HandleEdge(90, false);  // value 0
  decoder.HandleEdge(120, true);  // 2nd bit start
  decoder.HandleEdge(130, false); // value 0
  decoder.HandleEdge(160, true);  // 3rd bit start
  decoder.HandleEdge(170, false); // value 0
  decoder.HandleEdge(200, true);  // 4th bit start
  decoder.HandleEdge(210, false); // value 0
  decoder.HandleEdge(240, true);  // 5th bit start
  decoder.HandleEdge(250, false); // value 0
  decoder.HandleEdge(280, true);  // 6th bit start
  decoder.HandleEdge(290, false); // value 0
  decoder.HandleEdge(320, true);  // 7th bit start
  decoder.HandleEdge(330, false); // value 0
  decoder.HandleEdge(360, true);  // 8th bit start
  decoder.HandleEdge(370, false); // value 0
  decoder.HandleEdge(400, true);  // 9th bit start
  decoder.HandleEdge(410, false); // value 0
  decoder.HandleEdge(440, true);  // 10th bit start
  decoder.HandleEdge(450, false); // value 0
  decoder.HandleEdge(480, true);  // 11th bit start
  decoder.HandleEdge(490, false); // value 0
  decoder.HandleEdge(520, true);  // 12th bit start
  decoder.HandleEdge(530, false); // value 0
  decoder.HandleEdge(560, true);  // 13th bit start
  decoder.HandleEdge(570, false); // value 0
  decoder.HandleEdge(600, true);  // 14th bit start
  decoder.HandleEdge(610, false); // value 0
  decoder.HandleEdge(640, true);  // 15th bit start
  decoder.HandleEdge(650, false); // value 0
  decoder.HandleEdge(680, true);  // 15th bit start
  decoder.HandleEdge(690, false); // value 0
  decoder.HandleEdge(720, true);  // invalid pulse start
  decoder.HandleEdge(730, false); // invalid pulse
  decoder.HandleEdge(740, true);  // end of invalid pulse
  
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

  // when
  decoder.HandleEdge(0, true);    // sync start
  decoder.HandleEdge(40, false);  // sync
  decoder.HandleEdge(80, true);   // 1st bit start
  decoder.HandleEdge(110, false); // value 1
  decoder.HandleEdge(120, true);   // 2nd bit start
  decoder.HandleEdge(150, false); // value 1
  decoder.HandleEdge(160, true);  // invalid pulse start
  decoder.HandleEdge(170, false); // invalid pulse
  decoder.HandleEdge(180, true);  // end of invalid pulse
  
  // then
  byte expected[] = { 0x00 };
  dataReceiverMock.AssertHandleDataCalled(expected, 0);
}

void setup()
{
  Serial.begin(9600);
  while(!Serial) {} // Portability for Leonardo/Micro

  Test::min_verbosity |= TEST_VERBOSITY_ASSERTIONS_ALL;
}

void loop()
{
  Test::run();
}

