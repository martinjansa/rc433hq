#include "rc433hq.h"

class RC433HQEmosSocketsPulseDecoder: public RC433HQBasicSyncPulseDecoder {
public:
    RC433HQEmosSocketsPulseDecoder(IRC433DataReceiver &adataReceiver):
        RC433HQBasicSyncPulseDecoder(adataReceiver, 2947, 7300, 408, 1133, 907, 612, 100, true, 24, 24)
    {
    }
};