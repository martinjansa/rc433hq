#line 2 "rc433hq_emos_sockets_transmitter.ino"

//#define DEBUG

#if defined(ARDUINO)
#	include <rc433hq.h>
#	include <rc433hq_emos.h>
#else
#	include "../../rc433hq.h"
#	include "../../rc433hq_emos.h"
#endif // defined(ARDUINO)

class Logger: public IRC433Logger {
private:
  int logPin;
  bool logPinStatus;

public:
  Logger(int alogPin):
    logPin(alogPin),
    logPinStatus(false)
  {
    pinMode(logPin, OUTPUT);
  }

	virtual void LogMessage(const char *message)
  {
    //Serial.print(message);
    //logPinStatus = !logPinStatus;
    //digitalWrite(logPin, (logPinStatus? HIGH: LOW));
  }
};

// dumps bits from the given byte in their binary represenation. If bits are 8, starts with the higher bit 0x80, otherwise starts with lower bits
static char *ByteToBinary(byte value, size_t bits = 8) {
  static char buf[9];

  // assert (bits <= 8)

  // if not all the bits should be dumped
  if (bits < 8) {
    // shift the value left by the number of skipped bits
    value = value << (8 - bits);
  }

  size_t i;
  for (i = 0; i < bits; i++) {
    // write the bit character
    buf[i] = (((value & 0x80) != 0) ? '1' : '0');

    // shift the value
    value = value << 1;
  }
  // write the terminating zero
  buf[i] = '\0';
  return buf;
}

// logger, dumps log messages to serial
//Logger logger(LED_BUILTIN);

// the 433 MHz transmitter instance.
RC433HQTransmitter transmitter(10);

void setup()
{
  Serial.begin(9600);
  while(!Serial) {} // Portability for Leonardo/Micro
}

void SendEmosCode(const byte *data, size_t bits)
{
    RC433HQEmosSocketsPulseEncoderA encoderA;
    RC433HQEmosSocketsPulseEncoderB encoderB;

    // transmit the code

    // initialize the transmission without quiet period and with statistics
    RC433HQTransmissionQualityStatistics transmissionStats;
    transmitter.StartTransmission(0, 10, &transmissionStats);

    // transmit data according to the protocol A
    encoderA.EncodeData(transmitter, data, bits, 4);

    // transmit the same data according to the protocol B
    encoderB.EncodeData(transmitter, data, bits, 4);

    // finalize the transmission
    transmitter.EndTransmission(0);

    // dump the statistics
    Serial.print("Sent ");
    Serial.print(bits);
    Serial.print(" binary:");
    int i;
    for (i = 0; i < (bits >> 3); i++) {
      Serial.print(" ");
      Serial.print(ByteToBinary(data[i]));
    }
    Serial.print(", hexadecimal:");
    for (i = 0; i < (bits >> 3); i++) {
      Serial.print(" ");
      Serial.print(data[i], HEX);
    }
    Serial.print(", edges: ");
    Serial.print(transmissionStats.countOfTransmittedEdges);
    Serial.print(", delayed edges: ");
    Serial.print(transmissionStats.countOfDelayedEdges);
    Serial.print(", delayed outside tolerance: ");
    Serial.print(transmissionStats.countOfDelayedEdgesOutsideOfTolerance);
    Serial.print(", average delay: ");
    Serial.print(transmissionStats.averageDelay);
    Serial.print(" us.\n");
}

static const byte C_ON[] = { 0x38, 0xCB, 0xBE };
static const byte C_OFF[] = { 0x30, 0xFD, 0xAE };

void loop()
{
    SendEmosCode(C_ON, 24);
    delay(2000);
    SendEmosCode(C_OFF, 24);
    delay(2000);
}

