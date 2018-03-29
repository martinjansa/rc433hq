ARDUINO_UNIT_SRC_DIR=../../src
ARDUINO_UNIT_SRC=${ARDUINO_UNIT_SRC_DIR}/ArduinoUnitUtility/ArduinoUnitString.cpp ${ARDUINO_UNIT_SRC_DIR}/ArduinoUnitUtility/ArduinoUnit.cpp

rc433hq_tests : main.cpp rc433hq_tests.ino ${ARDUINO_UNIT_SRC}
	g++ -isystem ../../src -std=gnu++11 -o advanced main.cpp ${ARDUINO_UNIT_SRC}

