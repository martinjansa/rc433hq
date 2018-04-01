#line 2 "rc433hq_emos_sockets_receiver.ino"

#define DEBUG

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

class ReceivedDataDumper: public IRC433DataReceiver {
public:
  ReceivedDataDumper()
  {
  }
  virtual void HandleData(const byte *data, size_t bits)
  {
    // dump the received data 
    Serial.print("Received ");
    Serial.print(bits);
    Serial.print(" bits, binary: ");
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
    Serial.print("\n");
  }
};


// logger, dumps log messages to serial
Logger logger(LED_BUILTIN);

// the instance of the data dumper - prints all the data received via the HandleData()
ReceivedDataDumper recivedDataDumper;

// the instamce of the data decoder - decodes the EMOS Sockets protocol and hands the decoded data over to recivedDataDumper
RC433HQEmosSocketsPulseDecoder decoder(recivedDataDumper);

// buffer for handled data
RC433HQPulseBuffer buffer(decoder, 128);

// the instance of noise filter, that ignores all the very short pulses and passes the clean data to decoder
RC433HQNoiseFilter noiseFilter(buffer, 5);

// the 433 MHz receiver instance. Passes data to noise filter.
RC433HQReceiver receiver(noiseFilter, 2);

void setup()
{
  Serial.begin(9600);
  while(!Serial) {} // Portability for Leonardo/Micro

  decoder.SetLogger(logger);
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

