#include "rc433hq.h"

class RC433HQEmosSocketsPulseDecoder: public RC433HQBasicSyncPulseDecoder {
public:
    RC433HQEmosSocketsPulseDecoder(IRC433DataReceiver &adataReceiver):
        RC433HQBasicSyncPulseDecoder(adataReceiver, 272, 2381, 299, 1235, 1076, 480, 100, true, 24, 24)  // Part 1
//        RC433HQBasicSyncPulseDecoder(adataReceiver, 2948, 7302, 401, 1134, 918, 617, 100, true, 24, 24)  // Part 2
    {
    }
};