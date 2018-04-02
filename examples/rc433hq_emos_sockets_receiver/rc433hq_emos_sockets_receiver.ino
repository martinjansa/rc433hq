#line 2 "rc433hq_emos_sockets_receiver.ino"

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

class ReceivedDataDumper {
private:
  unsigned long lastDump;

public:
  ReceivedDataDumper():
    lastDump(0)
  {
  }

  virtual void DumpData(RC433HQMicroseconds time, const byte *data, size_t bits, double quality, const char *source)
  {
    // it it has been more than 1 s since the last dump
    if ((lastDump != 0) && ((time - lastDump) > (RC433HQMicroseconds(1000) * 1000))) {

      // separate the dumps by a new line
      Serial.print("\n");
    }

    lastDump = time;

    // dump the received data 
    Serial.print("Received ");
    Serial.print(bits);
    Serial.print(" bits binary:");
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
    Serial.print(", time: ");
    Serial.print(time);
    Serial.print(", source: ");
    Serial.print(source);
    Serial.print(", quality ");
    Serial.print(quality);
    Serial.print(" % ");
    for (i = 0; i < (int(quality) / 10); i++) {
      Serial.print("*");
    }
    Serial.print("\n");
  }
};


class ReceivedDataHandler: public IRC433DataReceiver {
private:
  ReceivedDataDumper &dumper;
  const char *source;
public:
  ReceivedDataHandler(ReceivedDataDumper &adumper, const char *asource):
    dumper(adumper),
    source(asource)
  {
  }
  virtual void HandleData(RC433HQMicroseconds time, const byte *data, size_t bits, double quality)
  {
    dumper.DumpData(time, data, bits, quality, source);
  }
};


// logger, dumps log messages to serial
Logger logger(LED_BUILTIN);

// the instance of the data dumper - prints all the data received via the HandleData()
ReceivedDataDumper recivedDataDumper;

ReceivedDataHandler handlerA(recivedDataDumper, "A");
ReceivedDataHandler handlerB(recivedDataDumper, "B");

// the instaces of the EMOS Socket data decoders for bot protocols A and B
RC433HQEmosSocketsPulseDecoderA decoderA(handlerA);
RC433HQEmosSocketsPulseDecoderB decoderB(handlerB);

// signal splitter to process the signal by both EMOS Socket processors A and B
RC433PulseSignalSplitter signalSplitter(decoderA, decoderB);

// buffer for handled data
RC433HQPulseBuffer buffer(signalSplitter, 256);

// the instance of noise filter, that ignores all the very short pulses and passes the clean data to decoder
RC433HQNoiseFilter noiseFilter(buffer, 3);

// the 433 MHz receiver instance. Passes data to noise filter.
RC433HQReceiver receiver(noiseFilter, 2);

// we will calculate and dump performance statistics every 10s
static const unsigned long STATS_PERIOD = 10*1000;

unsigned long startTimeMillis;

unsigned long iterationsCount = 0;
unsigned long totalProcessedCount = 0;
unsigned long totalMissedCount = 0;


void setup()
{
  Serial.begin(9600);
  while(!Serial) {} // Portability for Leonardo/Micro

  decoderA.SetLogger(logger);
  decoderB.SetLogger(logger);

  startTimeMillis = millis();
  iterationsCount = 0;
  totalProcessedCount = 0;
  totalMissedCount = 0;
}


void loop()
{
  // process the data in the buffer
  size_t reportedBufferUsedCount = 0;
  size_t reportedProcessedCount = 0;
  size_t reportedMissedCount = 0;
  buffer.ProcessData(reportedBufferUsedCount, reportedProcessedCount, reportedMissedCount);

  // add the counts to the statistics
  iterationsCount++;
  totalProcessedCount += reportedProcessedCount;
  totalMissedCount += reportedMissedCount;

  // if we've been processing data at least STATS_PERIOD
  if ((millis() - startTimeMillis) >= STATS_PERIOD) {

    // dump the statistics
    Serial.print("Data processing statistics: ");
    Serial.print(iterationsCount);
    Serial.print(" iterations, ");
    Serial.print(totalProcessedCount);
    Serial.print(" edges processed, ");
    Serial.print(totalMissedCount);
    Serial.print(" edges missed.\n");

    startTimeMillis = millis();
    iterationsCount = 0;
    totalProcessedCount = 0;
    totalMissedCount = 0;
  }
}

