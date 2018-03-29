#line 2 "rc433hq_tests.ino"
#include <string.h>
#include <ArduinoUnit.h>

// dummy test case
test(simple1)
{
  assertTrue(true);
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

