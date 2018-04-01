#include "rc433hq.h"

class RC433HQEmosSocketsPulseDecoder: public RC433HQBasicSyncPulseDecoder {
public:
    RC433HQEmosSocketsPulseDecoder(IRC433DataReceiver &adataReceiver):
        RC433HQBasicSyncPulseDecoder(adataReceiver, 2948, 7302, 401, 1134, 918, 617, 100, true, 24, 24)
    {
    }
};