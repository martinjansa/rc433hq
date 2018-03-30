#line 2 "rc433hq_tests.ino"
#include <string.h>
#include <ArduinoUnit.h>

#if defined(ARDUINO)
#	include <rc433hq.h>
#else
#	include "../../rc433hq.h"
#endif // defined(ARDUINO)

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

// dummy test case
test(ShouldDecodeOneCorrectBit)
{
  DataReceiverMock dataReceiverMock;
  RC433BasicSyncPulseDecoder decoder(dataReceiverMock, 40, 40, 10, 30, 30, 10, true, 1, 1);
  assertTrue(false);
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

