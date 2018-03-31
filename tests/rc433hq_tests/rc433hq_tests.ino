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
    for (size_t i = 0; i < storedBitsCount; i++) {
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

test(BasicPulseDecodeShouldDecodeOneExactOneBitFollowedBySync)
{
  // given
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 1);
  Logger logger;
  decoder.SetLogger(logger);

  // when
  decoder.HandleEdge(0, true);
  decoder.HandleEdge(40, false);
  decoder.HandleEdge(80, true);
  decoder.HandleEdge(110, false);
  decoder.HandleEdge(120, true);
  decoder.HandleEdge(160, false);
  decoder.HandleEdge(200, true);
  
  // then
  byte expected[] = { 0x01 };
  dataReceiverMock.AssertHandleDataCalled(expected, 1);
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

