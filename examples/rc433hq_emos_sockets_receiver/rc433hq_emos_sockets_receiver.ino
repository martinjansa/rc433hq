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

class ReceivedDataDumper {
public:
  virtual void DumpData(const byte *data, size_t bits, double quality, const char *source)
  {
    // dump the received data 
    Serial.print("Received ");
    Serial.print(bits);
    Serial.print(" bits, via source ");
    Serial.print(source);
    Serial.print(" binary: ");
    int i;
    for (i = 0; i < (bits >> 3); i++) {
      Serial.print(data[i], BIN);
      Serial.print(" ");
    }
    Serial.print(", hexadecimal: ");
    for (i = 0; i < (bits >> 3); i++) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print(", quality ");
    Serial.print(quality);
    Serial.print(" %\n");
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
  virtual void HandleData(const byte *data, size_t bits, double quality)
  {
    dumper.DumpData(data, bits, quality, source);
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
RC433HQPulseBuffer buffer(signalSplitter, 128);

// the instance of noise filter, that ignores all the very short pulses and passes the clean data to decoder
RC433HQNoiseFilter noiseFilter(buffer, 3);

// the 433 MHz receiver instance. Passes data to noise filter.
RC433HQReceiver receiver(noiseFilter, 2);

void setup()
{
  Serial.begin(9600);
  while(!Serial) {} // Portability for Leonardo/Micro

  decoderA.SetLogger(logger);
  decoderB.SetLogger(logger);
}

void loop()
{
  // process the data in the buffer
  size_t reportedUsedCount = 0;
  size_t reportedMissedCount = 0;
  buffer.ProcessData(reportedUsedCount, reportedMissedCount);

  /*
  // if some data were processed
  if (reportedUsedCount > 0 || reportedMissedCount > 0) {
    Serial.print("Data processing statistics: ");
    Serial.print(reportedUsedCount);
    Serial.print(" edges processed, ");
    Serial.print(reportedMissedCount);
    Serial.print(" edges missed.\n");
  }
  */
}

