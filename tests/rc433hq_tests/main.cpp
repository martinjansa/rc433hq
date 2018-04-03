#if !defined(ARDUINO)

// only used for "en vitro" tests (not on actual board)

#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

struct timeval starttime;

void setup();
void loop();

int main() {
  gettimeofday(&starttime, NULL);
  srand(time(0));
  setup();
  for (int i = 0; i < 2; i++) loop();
  return 0;
}

#define F(X) X
struct FakeSerial {
  void begin(long baud) { (void) baud; }
  bool operator!() const { return false; }
} Serial;


int random(int n) { return rand() % n; }
int random(int a, int b) { return a+rand() % (b-a+1); }

unsigned long millis() {
  struct timeval now;
  gettimeofday(&now, NULL);
  double secs = (double)(now.tv_usec - starttime.tv_usec) / 1000000 + (double)(now.tv_sec - starttime.tv_sec);
  return secs*1000;
}

#include "rc433hq_tests.ino"

// overloading streaming operator for debugging
std::ostream &operator<<(std::ostream &output, const RC433HQMicroseconds &that) { output << that.GetUnsignedLong(); return output; }
std::ostream &operator<<(std::ostream &output, const RC433HQMicrosecondsDiff &that) { output << that.GetUnsignedLong(); return output; }

bool operator<(const RC433HQMicroseconds &a, const RC433HQMicroseconds &b) { return a.GetUnsignedLong() < b.GetUnsignedLong(); }
bool operator<=(const RC433HQMicroseconds &a, const RC433HQMicroseconds &b) { return a.GetUnsignedLong() <= b.GetUnsignedLong(); }
bool operator>(const RC433HQMicroseconds &a, const RC433HQMicroseconds &b) { return a.GetUnsignedLong() > b.GetUnsignedLong(); }
bool operator>=(const RC433HQMicroseconds &a, const RC433HQMicroseconds &b) { return a.GetUnsignedLong() >= b.GetUnsignedLong(); }

#endif
