#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#define CLOCK_RATE_MHZ (uint64_t)180
#define CYCLES_PER_MICROSECOND CLOCK_RATE_MHZ
#define ONE_BILLION 1000000000

typedef struct
{
   int16_t biasPartsPerBillion;
   uint64_t timeCycles;
   int32_t remainderBillionths;
} CrystalSim ;

typedef struct
{
   int16_t adjustmentCyclesPerSecond;
   uint64_t timeMicroseconds;
   int16_t predivRemainderCycles;
   int16_t pendingAdjustment;
} AdjustableClock;

uint64_t advanceBiasedClockByMicroseconds(CrystalSim * this, uint32_t microseconds)
{
    int64_t stepByCycles = CYCLES_PER_MICROSECOND * microseconds;
    int64_t stepByBillionths = this->biasPartsPerBillion * CYCLES_PER_MICROSECOND * microseconds;
    int64_t billionths = stepByBillionths+this->remainderBillionths;
    int32_t cyclesFromBillionths = billionths / ONE_BILLION;
    int32_t remainderBillionths = billionths % ONE_BILLION;
    int64_t totalCyclesThisAdvance = stepByCycles+cyclesFromBillionths;
    
    this->timeCycles = this->timeCycles+totalCyclesThisAdvance;
    this->remainderBillionths = remainderBillionths;
    return totalCyclesThisAdvance;
}

uint64_t advanceAdjustableClockByCycles(AdjustableClock * this, uint64_t cycles)
{
    // In case the clock is run for more than 1ms
    while (cycles > 1000*CYCLES_PER_MICROSECOND) {
        advanceAdjustableClockByCycles(this, 1000*CYCLES_PER_MICROSECOND);
        cycles -= 1000*CYCLES_PER_MICROSECOND;
    }

    int64_t deltaCycles = cycles + this->pendingAdjustment;
    int64_t microseconds = (deltaCycles+this->predivRemainderCycles) / CYCLES_PER_MICROSECOND;
    int64_t predivRemainder = (deltaCycles+this->predivRemainderCycles) % CYCLES_PER_MICROSECOND;
    
    uint64_t newTimeMicroseconds = this->timeMicroseconds + microseconds;

    uint8_t tenMillisecondCycle = (newTimeMicroseconds / 10000) % 100;
    uint32_t prevTenMsCycle = (this->timeMicroseconds / 10000) % 100;

    int32_t cyclesPerSecond = this->adjustmentCyclesPerSecond;
    int32_t biasBaseValue = cyclesPerSecond/100;
    int32_t remainder = cyclesPerSecond-(100*biasBaseValue);
    if (remainder < 0) {
        biasBaseValue -= 1;
        remainder += 100;
    }
    uint8_t biasTenths = remainder / 10;
    uint8_t biasHundredths = remainder % 10;

    int32_t adjustment;
    if (tenMillisecondCycle != prevTenMsCycle) {
        adjustment = biasBaseValue;
        if (tenMillisecondCycle%10 < biasTenths)
            adjustment++;
        if (tenMillisecondCycle%10==9 && (tenMillisecondCycle/10) < biasHundredths)
            adjustment++;
    } else {
        adjustment = 0;
    }
    
    this->pendingAdjustment = adjustment;
    this->timeMicroseconds = newTimeMicroseconds;
    this->predivRemainderCycles = predivRemainder;
}

int main()
{
    {
        CrystalSim cs = {0};
        cs.biasPartsPerBillion = 1000;
        printf("CrystalSim test 1, before timeCycles: %" PRId64 "\n", cs.timeCycles);
        advanceBiasedClockByMicroseconds(&cs, 1000000);
        printf("CrystalSim test 1, after, timeCycles %" PRId64 "\n", cs.timeCycles);
    }

    {
        CrystalSim cs = {0};
        cs.biasPartsPerBillion = 1000;
        printf("CrystalSim test 2, before timeCycles: %" PRId64 "\n", cs.timeCycles);
        for (uint64_t i = 0 ; i<10000L ; i++)
        {
            advanceBiasedClockByMicroseconds(&cs, 100);
        }
        printf("CrystalSim test 2, after, timeCycles %" PRId64 "\n", cs.timeCycles);
    }

    {
        CrystalSim cs = {0};
        cs.biasPartsPerBillion = 1000;
        printf("CrystalSim test 3, before timeCycles: %" PRId64 "\n", cs.timeCycles);
        for (uint64_t i = 0 ; i<1000000L ; i++)
        {
            advanceBiasedClockByMicroseconds(&cs, 1);
        }
        printf("CrystalSim test 3, after, timeCycles %" PRId64 "\n", cs.timeCycles);
    }

    {
        AdjustableClock ac = {0};
        ac.adjustmentCyclesPerSecond = 180;

        printf("AdjustableClock test 1, before, timeMicroseconds %" PRId64 "\n", ac.timeMicroseconds);
        advanceAdjustableClockByCycles(&ac, 10*180000000L);
        printf("AdjustableClock test 1, after, timeMicroseconds %" PRId64 "\n", ac.timeMicroseconds);
    }

    {
        AdjustableClock ac = {0};
        ac.adjustmentCyclesPerSecond = 180;

        printf("AdjustableClock test 2, before, timeMicroseconds %" PRId64 "\n", ac.timeMicroseconds);
        for (uint64_t i = 0 ; i<18000000L ; i++)
        {
            advanceAdjustableClockByCycles(&ac, 100);
        }
        printf("AdjustableClock test 2, after, timeMicroseconds %" PRId64 "\n", ac.timeMicroseconds);
    }
    
    return 0;
}
