ARDUINO_UNIT_SRC_DIR=${USERPROFILE}/Documents/Arduino/libraries/ArduinoUnit/src
ARDUINO_UNIT_SRC=${ARDUINO_UNIT_SRC_DIR}/ArduinoUnitUtility/ArduinoUnitString.cpp ${ARDUINO_UNIT_SRC_DIR}/ArduinoUnitUtility/ArduinoUnit.cpp

rc433hq_tests: main.cpp rc433hq_tests.ino ${ARDUINO_UNIT_SRC} ..\..\rc433hq.cpp
	g++ -isystem ${ARDUINO_UNIT_SRC_DIR} -std=gnu++11 -o rc433hq_tests main.cpp ${ARDUINO_UNIT_SRC} ..\..\rc433hq.cpp

test:	rc433hq_tests
	./rc433hq_tests.exe

all:	test
