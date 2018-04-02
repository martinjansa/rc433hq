#include "rc433hq.h"

// EMOS Sockets send 4 repetitions of 24-bit using encoding A followed by 4 repetitions of the same 24 bits in encoding B.

// EMOS Sockets encoding A decoder
class RC433HQEmosSocketsPulseDecoderA: public RC433HQBasicSyncPulseDecoder {
public:
    RC433HQEmosSocketsPulseDecoderA(IRC433DataReceiver &adataReceiver):
        RC433HQBasicSyncPulseDecoder(adataReceiver, 272, 2381, 299, 1235, 1076, 480, 50, true, 24, 24)
    {
    }
};

// EMOS Sockets encoding B decoder
class RC433HQEmosSocketsPulseDecoderB: public RC433HQBasicSyncPulseDecoder {
public:
    RC433HQEmosSocketsPulseDecoderB(IRC433DataReceiver &adataReceiver):
        RC433HQBasicSyncPulseDecoder(adataReceiver, 2948, 7302, 401, 1134, 918, 617, 50, true, 24, 24)
    {
    }
};