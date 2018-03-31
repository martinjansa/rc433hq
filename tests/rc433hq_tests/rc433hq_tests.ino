#line 2 "rc433hq_tests.ino"
#include <string.h>
#include <ArduinoUnit.h>

#if defined(ARDUINO)
#	include <rc433hq.h>
#else
#	include "../../rc433hq.h"
#endif // defined(ARDUINO)

class PulseDecoderMock: public IRC433PulseDecoder {
private:
  unsigned long times[8];
  bool edges[8];
  size_t pos;
public:
	PulseDecoderMock(): pos(0) {}
	virtual void HandleEdge(unsigned long time, bool direction)
  {
    // is there is space in the iternal buffers
    if (pos < 8) {

      // store the incoming edge into the internal buffers
      times[pos] = time;
      edges[pos] = direction;
      pos++;
    }
  }

  void AssertHandleEdgeCalled(unsigned long expectedTimes[], bool expectedEdges[], size_t expectedEdgesCount)
  {
    assertEqual(pos, expectedEdgesCount);
    for (size_t i = 0; i < pos; i++) {
      assertEqual(times[i], expectedTimes[i]);
      assertEqual(edges[i], expectedEdges[i]);
    }
  }
};

/*
class DataReceiverMock: public IRC433DataReceiver {
private:
  bool handleDataCalled;
public:
  DataReceiverMock():
    handleDataCalled(false)
  {
  }
  virtual void HandleData(const byte *data, word bits)
  {
    handleDataCalled = true;
  }
  
};

test(ShouldDecodeOneCorrectBit)
{
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 1);
  
  assertTrue(false);
}
*/

test(NoiseFilterShouldForwardSlowPulse)
{  
  PulseDecoderMock mock;

  NoiseFilter filter(mock, 3);
  filter.HandleEdge(0, true);
  filter.HandleEdge(10, false);
  filter.HandleEdge(20, true);
  filter.HandleEdge(30, false);

  unsigned long expectedTimes[] = { 0, 10, 20 };
  bool expectedEdges[] = { true, false, true};
  mock. AssertHandleEdgeCalled(expectedTimes, expectedEdges, 3);
}

test(NoiseFilterShouldElimitateNoiseBeforeDownEdge)
{  
  PulseDecoderMock mock;

  NoiseFilter filter(mock, 3);
  filter.HandleEdge(0, true);
  filter.HandleEdge(9, false); // noise 
  filter.HandleEdge(9, true);  // noise 
  filter.HandleEdge(10, false);
  filter.HandleEdge(20, true);
  filter.HandleEdge(30, false);

  unsigned long expectedTimes[] = { 0, 10, 20 };
  bool expectedEdges[] = { true, false, true};
  mock. AssertHandleEdgeCalled(expectedTimes, expectedEdges, 3);
}

test(NoiseFilterShouldIgnoreTheSameDirectionEdge)
{  
  PulseDecoderMock mock;

  NoiseFilter filter(mock, 3);
  filter.HandleEdge(0, true);
  filter.HandleEdge(5, false); // noise 
  filter.HandleEdge(10, false);
  filter.HandleEdge(20, true);
  filter.HandleEdge(30, false);

  unsigned long expectedTimes[] = { 0, 10, 20 };
  bool expectedEdges[] = { true, false, true};
  mock. AssertHandleEdgeCalled(expectedTimes, expectedEdges, 3);
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

