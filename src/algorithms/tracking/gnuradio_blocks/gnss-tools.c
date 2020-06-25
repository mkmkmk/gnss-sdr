#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>


#ifndef GNSS_SIM
#include <sys/lock.h>
#include <ccproc.h>
#include <ccproc-gnss.h>
#include <ccproc-perfcnt.h>
#include <ccproc-csr.h>
#include <board.h>
#include <ccproc-utils.h>
#endif


#include "gnssSim.h"
//#include "gnss-sim-online.h"
#include "gnss-tools.h"
#include "print-dma.h"


//#define FREQ_MEAS_TICKS 100
#define FREQ_MEAS_TICKS 20

//#warning temp!!
//#define FREQ_MEAS_TICKS 1

// czy sprawdzać czy nie jest nastawiony trigger przy zapisach do free-accu
#define DBG_CHECK_FREE_ACCU (0)



static const uint64_t Q32 = 0x100000000;
static const uint64_t Q16 = 0x10000;


#ifdef PRE_CARR_REM4_AND_DISC0_PROC
static int prevPllDisc[MAX_CHAN];
static int prevDllDisc[MAX_CHAN];
#endif


//#define barrier() __asm__ __volatile__("": : :"memory")
#define barrier() MEMORY_BARRIER();

// kopia GNSS_FREE_ACCU
// (zgodna tylko gdy jedyny dostęp będzie przez gnss-tools)
volatile static uint32_t _GNSS_FREE_ACCU_SHADOW = 0;

// globalne zabezpieczenie przed mod free accu, gdy ustawiony trigger
// zadziała tylko gdy jedyny dostęp będzie przez gnss-tools
volatile static uint64_t _GNSS_FREE_ACCU_NEXT_TRIGGER_END = 0;

int moduloDiff(int a, int b, int codeLen)
{
    int v1 = (a - b + codeLen) % codeLen;
    int v2 = (b - a + codeLen) % codeLen;
    return v1 < v2 ? v1 : v2;

}


void dbg_prog_stop()
{
    printdma("# PROG STOP!\n");
    printdma_wait();
    pause_interrupts();
    while(1)
    {
    }
}


volatile uint64_t getPerfCounter()
{
    //#ifndef GNSS_SIM
	gnss_sim_perf_counter_lock();
    volatile uint64_t perf_counter = PERFCNT_PTR->CYCLE_LO;
    perf_counter |= (uint64_t)PERFCNT_PTR->CYCLE_HI << 32;
    gnss_sim_perf_counter_unlock();
    return perf_counter;
    //#else
    //    return 0;
    //#endif
}

void sleep_until_c(uint64_t next)
{
#ifndef GNSS_SIM
    volatile uint64_t inext = next;
    while(getPerfCounter() < inext)
    {
        __asm__ volatile("nop;nop;nop;nop;nop;");
    }

#endif

}

void sleep_c(uint64_t cycles)
{
    sleep_until_c(getPerfCounter() + cycles);
}


#ifdef CPU_CCNV1_A1
static int _current_range_channel = 0;
#endif

// zadziała dla CCNV1_A1 tylko gdy wywoływana zawsze jako pierwsza zamiast GNSS_CODE_RNG()
uint32_t gnss_code_rng_a1_safe(int32_t channel)
{
#ifndef CPU_CCNV1_A1
    return GNSS_CODE_RNG(channel);
#else
    if(channel >= 0 && channel < 64)
        _current_range_channel = channel;
    return GNSS_CODE_RNG(_current_range_channel);
#endif
}


void readRangeReg(int rangeChannel, volatile uint64_t *tickTm, volatile uint32_t *rangeReg)
{
#if 0
    volatile int en = are_irq_enabled();
    if(en) pause_interrupts();

    //wyzerowanie rng_state
    GNSS_ACCU_ADD(0, 0);
    volatile uint32_t rng = gnss_code_rng_a1_safe(rangeChannel);
    volatile uint64_t ts = GNSS_CODE_RNG(0);
    ts |= (uint64_t)GNSS_CODE_RNG(0) << 32;
    if (en) resume_interrupts();

    if(tickTm)
        *tickTm = ts;

    if(rangeReg)
        *rangeReg = rng;
#else
    readRangeRegPh(rangeChannel, tickTm, rangeReg, 0);
#endif
}


void readRangeRegPh(int rangeChannel, volatile uint64_t *tickTm, volatile uint32_t *rangeReg, volatile uint32_t *carrCycles)
{
    volatile uint32_t iCarrCycles;
    volatile int en = are_irq_enabled();
    if(en) pause_interrupts();

    // wyzerowanie rng_state
    GNSS_ACCU_ADD(0, 0);
    volatile uint32_t rng = gnss_code_rng_a1_safe(rangeChannel);
    volatile uint64_t ts = GNSS_CODE_RNG(0);
    ts |= (uint64_t)GNSS_CODE_RNG(0) << 32;
#ifndef CPU_CCNV1_A1
    // dla A1 to zmieniłoby też kanał bazowy!
    // if(carrCycles)
    iCarrCycles = GNSS_CODE_RNG(0);
#else
    iCarrCycles = 0;
#endif
    if (en) resume_interrupts();

    if(tickTm)
        *tickTm = ts;

    if(rangeReg)
        *rangeReg = rng;

    if(carrCycles)
        *carrCycles = iCarrCycles;
}


volatile uint64_t getTickTime()
{
    volatile uint64_t tick_tm;
    readRangeReg(-1, &tick_tm, 0);
    return tick_tm;
}

// obsolete
int measClockFreq(Track *conf, volatile int *irq_chan_ticks/*, _LOCK_T *int4_lock*/)
{
    return measClockFreq_ex(conf, irq_chan_ticks, FREQ_MEAS_TICKS);
}

// obsolete
int oldReadTicks(void* stateMem, int channel)
{
    volatile int *irq_chan_ticks = (int*)stateMem;

#ifndef REAL_ADC_MODE
#ifdef GNSS_SIM
        GNSS_TRACK_STEP(0);
        GNSS_CHANN_SET(0);
        if (IS_PTICK(GNSS_CODE_GET()))
            irq_chan_ticks[0]++;
#endif
#endif

    pause_interrupts();
    int ch_tick = irq_chan_ticks[channel];
    irq_chan_ticks[channel] = 0;
    resume_interrupts();
    return ch_tick;
}


int measClockFreq_ex2(Track *conf, void* stateMem, int (*isrReadTicks)(void* stateMem, int channel), int freqMeasTicks);

// obsolete
int measClockFreq_ex(Track *conf, volatile int *irq_chan_ticks, int freqMeasTicks)
{
    return measClockFreq_ex2(conf, (void*)irq_chan_ticks, oldReadTicks, freqMeasTicks);
}

int innerAdaptReadTicks(void* stateMem, int channel)
{
    int (*isrReadTicks)(int) = (int (*)(int))stateMem;
    return isrReadTicks(channel);
}

int measClockFreqNew(Track *conf, int (*isrReadTicks)(int channel))
{
    return measClockFreq_ex2(conf, isrReadTicks, innerAdaptReadTicks, FREQ_MEAS_TICKS);
}

int measClockFreqNewEx(Track *conf, int (*isrReadTicks)(int channel), int freqMeasTicks)
{
    return measClockFreq_ex2(conf, isrReadTicks, innerAdaptReadTicks, freqMeasTicks);
}


//meas clock, needs one channel tick-ing
int measClockFreq_ex2(Track *conf, void* stateMem, int (*isrReadTicks)(void* stateMem, int channel), int freqMeasTicks)
{
    printdma("#clock meas...\n");
    int meas_clk_freq = PERF_COUNTER_FREQ;
#if !defined(REAL_ADC_MODE) && !defined(GNSS_SIM)
    printdma("#meas_clk_freq[Hz]=%d\n", meas_clk_freq);
    return meas_clk_freq;
#endif

    uint64_t prev_watch = 0;
    __attribute__((unused))  uint64_t prev_meas = 0;

    int tick = 0;
	
	while (1)
    {

//#ifndef REAL_ADC_MODE
//#ifdef GNSS_SIM
//        GNSS_TRACK_STEP(0);
//        GNSS_CHANN_SET(0);
//        if (IS_PTICK(GNSS_CODE_GET()))
//            irq_chan_ticks[0]++;
//#endif
//#endif

#ifdef TICK_PULLING
        //*lastTickFlags = GNSS_PTR->TICKF;
        //GNSS_PTR->TICKF = 0xFFFF;
        volatile uint32_t iLastTickF = GNSS_PTR->TICKF;
        if (BitGet(iLastTickF, conf->channel))
        {
            irq_chan_ticks[conf->channel]++;
        }
        if (iLastTickF)
        {
            //GNSS_PTR->TICKF = 0xFFFF;
            GNSS_PTR->TICKF = iLastTickF;
        }

#endif
        //pause_interrupts();
        ////__lock_acquire(*int4_lock);
        //int ch_tick = irq_chan_ticks[conf->channel];
        //irq_chan_ticks[conf->channel] = 0;
        ////__lock_release(*int4_lock);
        //resume_interrupts();
        int ch_tick = isrReadTicks(stateMem, conf->channel);

        if (!ch_tick)
        {
            volatile int64_t currPerf = getPerfCounter();
            if (currPerf - prev_watch > PERF_COUNTER_FREQ)
            {
                printdma("#[measClock] no ticks\n");
                prev_watch = currPerf;
            }
            continue;
        }

        GNSS_CHANN_SET(conf->channel);
        volatile uint64_t perf_counter0 = getTickTime();

        if (tick == freqMeasTicks)
        {
            meas_clk_freq = (perf_counter0 - prev_meas) * conf->fsample / conf->seqLen / freqMeasTicks;
            printdma("#meas_clk_freq[Hz]=%d\n", meas_clk_freq);
            break;
        }

        if (tick == 0)
            prev_meas = perf_counter0;

        tick++;
    }

    return meas_clk_freq;
}




void pause_free_accu_at(int cyclesToSkip, uint64_t when)
{
    if (cyclesToSkip < 3)
    {
        printdma("\tlow pause!!\t");
        return;
    }

    volatile uint64_t iwhen = (uint64_t)when;
    volatile uint64_t inext = (uint64_t)when + cyclesToSkip;

    if (iwhen == 0 || inext == 0)
    {
        printdma("\n *** pause_free_accu_at(): zero trigger (%u, %u) \n", iwhen, inext);
        return;
    }
    _GNSS_FREE_ACCU_NEXT_TRIGGER_END = inext;

    volatile int en = are_irq_enabled();
    if(en) pause_interrupts();

    //nr kanału wybierany bitami w pierwszym argumencie!
    //ale dotyczy tylko bież kanału dlatego 0xFFFF może być
    GNSS_FREE_ACCU_WR(0, iwhen);
    GNSS_FREE_ACCU_WR(0xFFFF, inext);
    if(en) resume_interrupts();

    volatile uint64_t tm = getPerfCounter();
    //low marg
    if (tm > iwhen)
    {
        printdma("\n# *** LOW-MARG-PAUSE (%d) !!\a\n", (int) (tm - iwhen));
        //BREAKPOINT();
    }
    //printdma("{p}");
}


void pause_free_accu(int cpu_cycles/*, _LOCK_T *int4_lock*/)
{
    // pauzowanie przerwań może trwać 150 cykli
    // dlatego lepiej żeby pomiar czasu był po pauzie przerwań
    volatile int en = are_irq_enabled();
    if(en) pause_interrupts();
#ifdef BOARD_CCNV1_A1
    int marg = 1000;
#else
    int marg = 300;
#endif
    volatile uint64_t when = getPerfCounter() + marg;
    pause_free_accu_at(cpu_cycles, when);
    if (en) resume_interrupts();
}


void pause_free_accu_multi_at(int cpu_cycles, uint64_t when, uint32_t channels_mask)
{
    _GNSS_FREE_ACCU_NEXT_TRIGGER_END = when;
    volatile int en = are_irq_enabled();
    if(en) pause_interrupts();

    int prev_ch = GNSS_CHANN_GET();
    int bit = 1;
    for (int ch = 0; ch < MAX_CHAN; ch++)
    {
        if (channels_mask & bit)
        {
            GNSS_CHANN_SET(ch);
            //nr kanału wybierany bitami w pierwszym argumencie!
            //ale dotyczy tylko bież kanału dlatego 0xFFFF może być
            GNSS_FREE_ACCU_WR(0, when);
            GNSS_FREE_ACCU_WR(0xFFFF, when + cpu_cycles);
        }
        bit <<= 1;

    }
    volatile uint64_t tm = getPerfCounter();
    GNSS_CHANN_SET(prev_ch);
    if (en) resume_interrupts();

    //low marg
    if (tm > when) printdma("\tL-MG-MT !!\t");

    //printdma(" LEFT:%d ", (int) (when - tm));
}


void pause_free_accu_multi(int cpu_cycles, uint32_t channels_mask)
{
    // pauzowanie przerwań może trwać 150 cykli
    // dlatego lepiej żeby pomiar czasu był po pauzie przerwań
    volatile int en = are_irq_enabled();
    if(en) pause_interrupts();

    int marg = 600;
    volatile uint64_t tm = getPerfCounter();
    pause_free_accu_multi_at(cpu_cycles, tm + marg, channels_mask);

    if (en) resume_interrupts();

}


void enableGnssInterupts()
{
#ifndef GNSS_SIM

#ifndef TICK_PULLING
    // enable GNSS_IRQn interrupt
    CSR_CTRL_PTR->IRQ_MASK |= (1 << GNSS_IRQn);
    CSR_CTRL_PTR->STATUS |= CSR_STAT_CIEN;
#endif
    // tick interrupt enable
    GNSS_PTR->CTRL |= GNSS_CTRL_CODE_TICK_IE;
    GNSS_PTR->CTRL |= GNSS_CTRL_CODE_OVR_IE;

    printdma("PTICK IRQ ENABLED\n");

#endif
}


volatile int are_irq_enabled()
{
    return CSR_CTRL_PTR->STATUS & CSR_STAT_CIEN;
}


void pause_interrupts()
{
    //gnss_sim_irq_block();
    //gnss_sim_online_lock();
    CSR_CTRL_PTR->STATUS &= ~CSR_STAT_CIEN;
    //gnss_sim_online_unlock();
    barrier();
}


void resume_interrupts()
{
    barrier();
    //gnss_sim_online_lock();
    CSR_CTRL_PTR->STATUS |= CSR_STAT_CIEN;
    //gnss_sim_online_unlock();
    //gnss_sim_irq_unblock();
}


void FilterCoefs(float K, float Bn, int sumTimeMs, int *c1, int *c2)
{
    float wn = Bn / .53f;
    float zeta = 0.707f;
    float t1 = K / wn / wn;
    float t2 = 2 * zeta / wn;
    float C2 = sumTimeMs / 1000.0f / t1;
    float C1 = t2 / t1;

    *c1 = (int) (.5f + C1);
    *c2 = (int) (.5f+ C2);
}


//todo parametry do wywłołania, reszta do this
void setupChannel(int channel, Track *track, int mixMode)
{

    //printdma("Setup channel %d \n", channel);

    track->channel = channel;

    track->sumtime_ms = 0.5f + (track->pCodeLen * 1000.0f) / track->chipRate;
    //printdma("sumTime [ms] : %d \n", track->sumtime_ms);

    // Kpll = fsample * pi / 2 ^ 31 * 128 * 180 / pi
    // Kpll = fsample / 2 ^ 31 * 128 * 180
    track->Kpll = (float) track->fsample / Q32 * track->codeChipDiv * 128 * 180 / Q16;

    FilterCoefs(track->Kpll, track->pllBn, track->sumtime_ms, &track->pllC1, &track->pllC2);
    //printdma("comp PLL coefs : Bn=%d, C1=%d, C2=%d \n", track->pllBn, track->pllC1, track->pllC2);

    // Kdll = fsample * 2^30 * 2 / half_chip / 2^32
    // Kdll = fsample * 2^30 * 2 / 2^32 / 2^32
    // Kdll = fsample / 2 / 2^32
    track->Kdll = (float) track->fsample / Q32 / 2;
    FilterCoefs(track->Kdll, track->dllBn, track->sumtime_ms, &track->dllC1, &track->dllC2);
    //printdma("comp DLL coefs : Bn=%d, C1=%d, C2=%d \n", track->dllBn, track->dllC1, track->dllC2);

    uint32_t codeStep = (Q32 * track->codeChipDiv) * track->chipRate / track->fsample;
    uint32_t carrStep = Q32 * abs(track->freq) / track->fsample;

    GNSS_CHANN_SET(channel);
    GNSS_CARR_FREQ(carrStep);

//#ifdef PRE_CARR_REM4_AND_DISC0_PROC
//  GNSS_CARR_SET(0, track->freq < 0);
//#else
//  //wersja dla LAB-SAT, docel może być inny!
//  GNSS_CARR_SET(0, track->freq > 0 ? 1 : 3);
//  //#warning MIESZANIE MAX-A
//  //GNSS_CARR_SET(0, 3);
//
//#endif
//

    GNSS_CARR_SET(0, mixMode);

    GNSS_CARR_REM(0, 0);
    GNSS_CARR_REM(0, 0);

    GNSS_CARR_DISC(0);
    GNSS_CODE_DISC(0);

    GNSS_CODE_NCO_FREQ(codeStep, track->bocDiv);
    GNSS_CODE_EPL_FREQ(1, track->codeChipDiv);

    GNSS_PCODE_LEN(track->pCodeLen, 0, 0);
    GNSS_SCODE_LEN(track->sCodeLen);
    GNSS_PCODE_ADDR_SET(0, 0);
    GNSS_SCODE_ADDR_SET(0);

    int cnt = track->pCodeLen / 32;
    if (track->pCodeLen % 32 > 0)
        cnt++;
    for (int i = 0; i < cnt; i++)
        GNSS_PCODE_WR(track->pCode[i]);

    cnt = track->sCodeLen / 32;
    if (track->sCodeLen % 32 > 0)
        cnt++;
    for (int i = 0; i < cnt; i++)
        GNSS_SCODE_WR(track->sCode[i]);

    // GNSS_PLL_FLT_COEF(track->pllC1, track->pllC2);
    GNSS_PLL_FLT_COEF(track->pllC1, track->pllC2);
    GNSS_PLL_FLT_RST();

    GNSS_DLL_FLT_COEF(track->dllC1, track->dllC2);
    GNSS_DLL_FLT_RST();


#ifdef PRE_CARR_REM4_AND_DISC0_PROC
    for(int ch = 0; ch < MAX_CHAN; ch++)
    {
        prevPllDisc[ch] = 0;
        prevDllDisc[ch] = 0;
    }
#endif
}


//static void track_set_gnss_ise(Track *self, int mixMode);


void track_setup(
        Track *self,
        int channel,
        int sig_freq,
        int fsample,
        int chipRate,
        uint32_t *pCode,
        int pCodeLen,
        uint32_t *sCode,
        int sCodeLen,
        int seqLen,
        int codeChipDiv,
        int bocDiv,
        int mixMode,
        int pllBn,
        int dllBn,
        int seq_shift_samp
        )
{
    //printdma("Setup channel %d \n", channel);

    self->channel = channel;
    self->freq = sig_freq;
    self->fsample = fsample;
    self->codeChipDiv = codeChipDiv;
    self->chipRate = chipRate;
    self->pCodeLen = pCodeLen;
    self->pCode = pCode;
    self->sCodeLen = sCodeLen;
    self->sCode = sCode;
    self->seqLen = seqLen;
    self->bocDiv = bocDiv;
    self->pllBn = pllBn;
    self->dllBn = dllBn;
    self->seq_shift_samp = seq_shift_samp;


    self->sumtime_ms = 0.5f + (self->pCodeLen * 1000.0f) / self->chipRate;
    printdma("sumTime [ms] : %d \n", self->sumtime_ms);


    // Kpll = fsample * pi / 2 ^ 31 * 128 * 180 / pi
    // Kpll = fsample / 2 ^ 31 * 128 * 180
    self->Kpll = (float)self->fsample / Q32 * self->codeChipDiv * 128 * 180 / Q16;

    FilterCoefs(self->Kpll, self->pllBn, self->sumtime_ms, &self->pllC1, &self->pllC2);
    //printdma("comp PLL coefs : Bn=%d, C1=%d, C2=%d \n", self->pllBn, self->pllC1, self->pllC2);

    // Kdll = fsample * 2^30 * 2 / half_chip / 2^32
    // Kdll = fsample * 2^30 * 2 / 2^32 / 2^32
    // Kdll = fsample / 2 / 2^32
    self->Kdll = (float)self->fsample / Q32 / 2;
    FilterCoefs(self->Kdll, self->dllBn, self->sumtime_ms, &self->dllC1, &self->dllC2);
    //printdma("comp DLL coefs : Bn=%d, C1=%d, C2=%d \n", self->dllBn, self->dllC1, self->dllC2);


//    track_set_gnss_ise(self, mixMode);
//}
//static void track_set_gnss_ise(Track *self, int mixMode)
//{

    uint32_t codeStep = (Q32 * self->codeChipDiv) * self->chipRate / self->fsample;
    uint32_t carrStep = Q32 * abs(self->freq) / self->fsample;

    GNSS_CHANN_SET(self->channel);
    GNSS_CARR_FREQ(carrStep);


    //#ifdef PRE_CARR_REM4_AND_DISC0_PROC
    //  GNSS_CARR_SET(0, track->freq < 0);
    //#else
    //  //wersja dla LAB-SAT, docel może być inny!
    //  GNSS_CARR_SET(0, track->freq > 0 ? 1 : 3);
    //  //#warning MIESZANIE MAX-A
    //  //GNSS_CARR_SET(0, 3);
    //#endif

    GNSS_CARR_SET(0, mixMode);


    GNSS_CARR_REM(0, 0);
    GNSS_CARR_REM(0, 0);

    GNSS_CARR_DISC(0);
    GNSS_CODE_DISC(0);


    GNSS_CODE_NCO_FREQ(codeStep, self->bocDiv);
    GNSS_CODE_EPL_FREQ(1, self->codeChipDiv);

    GNSS_PCODE_LEN(self->pCodeLen, 0, 0);
    GNSS_SCODE_LEN(self->sCodeLen);
    GNSS_PCODE_ADDR_SET(0, 0);
    GNSS_SCODE_ADDR_SET(0);

    int cnt = (self->pCodeLen + 31) / 32;
    for (int i = 0; i < cnt; i++)
        GNSS_PCODE_WR(self->pCode[i]);

    if(self->sCodeLen)
    {
        cnt = (self->sCodeLen + 31) / 32;
        for (int i = 0; i < cnt; i++)
            GNSS_SCODE_WR(self->sCode[i]);
    }

    // GNSS_PLL_FLT_COEF(track->pllC1, track->pllC2);
    GNSS_PLL_FLT_COEF(self->pllC1, self->pllC2);
    GNSS_PLL_FLT_RST();

    GNSS_DLL_FLT_COEF(self->dllC1, self->dllC2);
    GNSS_DLL_FLT_RST();

    //todo to chyba tu nie powinno być
    //setCodeShift(self, self->seq_shift_samp);


#ifdef PRE_CARR_REM4_AND_DISC0_PROC
    for(int ch = 0; ch < MAX_CHAN; ch++)
    {
    prevPllDisc[ch] = 0;
    prevDllDisc[ch] = 0;
    }
#endif
}


static inline uint32_t BitSetVal(uint32_t arg, uint32_t pos, uint32_t val)
{
    return val ? BitSet(arg, pos) : BitClr(arg, pos);
}


static volatile uint64_t _prevFreeAccuSetSafeTime = 0;


int isFreeAccuSetSafeEx(uint64_t time)
{
    // gdy przekręcenie zegara
    if (time <= _prevFreeAccuSetSafeTime)
        _GNSS_FREE_ACCU_NEXT_TRIGGER_END = 0;
    _prevFreeAccuSetSafeTime = time;
    if (time >= _GNSS_FREE_ACCU_NEXT_TRIGGER_END)
        _GNSS_FREE_ACCU_NEXT_TRIGGER_END = 0;

    return _GNSS_FREE_ACCU_NEXT_TRIGGER_END == 0; //|| time > _GNSS_FREE_ACCU_NEXT_TRIGGER_END;
}


int isFreeAccuSetSafe()
{
    volatile uint64_t tm = getPerfCounter();
    return isFreeAccuSetSafeEx(tm);
}

static inline uint32_t checkedFreeAccuRd()
{
    volatile uint32_t rd = GNSS_FREE_ACCU_RD();
    if (rd != _GNSS_FREE_ACCU_SHADOW)
    {
        if (DBG_CHECK_FREE_ACCU)
        {
            printdma("\n# GNSS_FREE_ACCU_RD err (%08x, %08x)\n", rd, _GNSS_FREE_ACCU_SHADOW);
            dbg_prog_stop();
        }
        rd = _GNSS_FREE_ACCU_SHADOW;
    }
    return rd;
}


void setFreeAccu(int channel, int isOn)
{
    if (DBG_CHECK_FREE_ACCU && !isFreeAccuSetSafe())
    {
        printdma("\n# *** NOT SAFE setFreeAccu (acc)!! {%dc} \a \n", (int) (_GNSS_FREE_ACCU_NEXT_TRIGGER_END - _prevFreeAccuSetSafeTime));
        dbg_prog_stop();
    }
    volatile uint32_t rd = checkedFreeAccuRd();
    volatile uint32_t wr = BitSetVal(rd, channel, isOn);
    GNSS_FREE_ACCU_WR(wr, 0);
    _GNSS_FREE_ACCU_SHADOW = wr;
    _GNSS_FREE_ACCU_NEXT_TRIGGER_END = 0;
    //rd = GNSS_FREE_ACCU_RD();
    //printdma("#F-AC=0x%08X\n", (unsigned)rd);
    //printdma("#%d|facu %d\n", channel, isOn);
    //printdma("#%d|facu %d (%s)\n", channel, isOn, rd == _GNSS_FREE_ACCU_SHADOW ? "OK" : "ERROR");
}


volatile int getFreeAccu(int channel)
{
    volatile uint32_t rd = checkedFreeAccuRd();
    return BitGet(rd, channel);
}


void setFreeAccuVel(int channel, int vel)
{
    if (DBG_CHECK_FREE_ACCU && !isFreeAccuSetSafe())
    {
        printdma("\n# *** NOT SAFE setFreeAccu (vel) !! {%dc} \a \n", (int) (_GNSS_FREE_ACCU_NEXT_TRIGGER_END - _prevFreeAccuSetSafeTime));

        dbg_prog_stop();
    }

    volatile uint32_t rd = checkedFreeAccuRd();
    volatile uint32_t wr = BitSetVal(rd, 16 + channel, vel);

    GNSS_FREE_ACCU_WR(wr, 0);
    _GNSS_FREE_ACCU_SHADOW = wr;
    //rd = GNSS_FREE_ACCU_RD();
    //printdma("#F-AC=0x%08X\n", (unsigned)rd);
    //printdma("#%d|vel %d\n", channel, vel);
    //printdma("#%d|vel %d (%s)\n", channel, vel, rd == _GNSS_FREE_ACCU_SHADOW ? "OK" : "ERROR");
}


void setFreeUpdate(int channel, int isOn)
{
    setFreeUpdateEx(channel, isOn, isOn);
}


void setFreeUpdateEx(int channel, int pll, int dll)
{
    volatile uint32_t reg = GNSS_FREE_UPDATE_RD();
    reg = BitSetVal(reg, channel, pll);
    reg = BitSetVal(reg, 16 + channel, dll);
    GNSS_FREE_UPDATE_WR(reg);
    //printdma("#%d|PLL:%d,DLL:%d\n", channel, pll, dll);
}


void getFreeUpdate(int channel, int *pll, int *dll)
{
    volatile uint32_t rd = GNSS_FREE_UPDATE_RD();
    *pll = BitGet(rd, channel);
    *dll = BitGet(rd, 16 + channel);
}


void setAfe(int chan, int val)
{
    volatile uint32_t reg = GNSS_AFE_RD();
    int shift = chan << 1;
    reg = (reg & ~(3 << shift)) | ((val & 3) << shift);
    GNSS_AFE_WR(reg);
}


int getAfe(int chan)
{
    volatile uint32_t reg = GNSS_AFE_RD();
    int shift = chan << 1;
    return (reg >> shift) & 3;
}


void setCodeShift(Track *track, int toShift)
{
    int channel = track->channel;

    GNSS_CHANN_SET(channel);

    GNSS_PCODE_ADDR_SET(0, 0);
    GNSS_SCODE_ADDR_SET(0);

    //int toShift = track->seq_shift_samp % track->seqLen;
    toShift %= track->seqLen;

	int accuConf = GNSS_FREE_ACCU_RD();

    int obsoShift = track->bocDiv;

    if(obsoShift)
    {
        //TODO BOC GNSS_PCODE_ADDR_SET not implemented
		setFreeAccu(channel, 0);

		for (int i = 0; i < toShift; i++)
		{
			GNSS_CODE_GET();
		}
    }
    else
    {
		//unsigned addr = (uint64_t)toShift * (track->chipRate / 1000) / (track->fsample / 1000);   //toShift / chip;
		//unsigned chip_ptr = (toShift * (track->chipRate / 1000) / track->seqLen * track->codeChipDiv) % track->codeChipDiv;
		//code_nco = (toShift / (chip / track->codeChipDiv) * Q32) % Q32
		//unsigned code_nco = (unsigned)(((uint64_t)toShift * (track->chipRate / 1000) / track->seqLen * track->codeChipDiv) << 32);

		//TODO BOC_PTR !!
		float chip = (float)toShift * track->chipRate / 1000 / track->seqLen;
		unsigned addr = chip;
		float chipFrac = chip - truncf(chip);
		unsigned code_nco = (unsigned)(chipFrac * track->codeChipDiv * Q32);
		unsigned chip_ptr = (unsigned)(chipFrac * track->codeChipDiv);
		GNSS_PCODE_ADDR_SET(addr | (chip_ptr << 24), code_nco);
    }

    if (track->sCodeLen > 0)
    {
        GNSS_SCODE_ADDR_SET(track->nh_idx % track->sCodeLen);
    }

    if(obsoShift)
    {
    	GNSS_FREE_ACCU_WR(accuConf, 0);
    	_GNSS_FREE_ACCU_SHADOW = accuConf;
        while(1){}
    }

}

#if 0

static inline int compl4b(uint32_t val)
{
  val &= 0xF;
  return (val < 8) ? val : val - 16;
}

static inline void decodeIQ4b(uint8_t out, int* sigI, int* sigQ)
{
  *sigI = compl4b(out & 0xF);
  *sigQ = compl4b((out >> 4) & 0xF);
}


static inline void printCodeStep(uint32_t val)
{
  printdma("|%2d ", (int) ((val >> 31) & 0x1));
  printdma("%2d : ", (int) ((val >> 30) & 0x1));
  printdma("%2d ", (int) compl4b((val >> 12) & 0xF));
  printdma("%2d ", (int) compl4b((val >> 4) & 0xF));
  printdma("%2d ", (int) compl4b((val >> 0) & 0xF));
  printdma("%2d ", (int) compl4b((val >> 8) & 0xF));
  printdma("%2d |", (int) compl4b((val >> 16) & 0xF));
}

#endif


void track_step(int sampI, int sampQ)
{
  int inp = (sampI & 0xFFFF) | ((sampQ & 0xFFFF) << 16);
  GNSS_TRACK_STEP(inp);
  
}


void readChannel(int channel, int *ptick_out, int* accu_out)
{

  GNSS_CHANN_SET(channel);


  int32_t code = GNSS_CODE_GET();

  int ptick = IS_PTICK(code);
  int doPrint = 0 & ptick;

  *ptick_out = ptick; 

  if (ptick)
  {
    int accu = GNSS_ACCU_GET(); // empty
    
    if (doPrint) printdma("%6d; ", channel);
    
    for (int j = 0; j < 10; j++)
    {
      accu = GNSS_ACCU_GET();
      accu_out[j] = accu;
      if (doPrint)
      {
        printdma("%6d; ", (int) accu);
      }
    }
  }

  if (doPrint)
  {
    printdma("\n");
  }

}



void loopFeedback_ex(int channel, int iP, int qP, int iE, int qE, int iL, int qL, int *pllDisc_out, int *dllDisc_out)
{

  GNSS_CHANN_SET(channel);
  
  int pllDisc = GNSS_COST_DISC(iP, qP);
  int pllDiscFlt = GNSS_PLL_FLT(pllDisc);
  *pllDisc_out = pllDiscFlt;

#ifdef PRE_CARR_REM4_AND_DISC0_PROC
  GNSS_CARR_DISC(pllDiscFlt - prevPllDisc[channel]);
  prevPllDisc[channel] = pllDiscFlt;
#else
  GNSS_CARR_DISC(pllDiscFlt);
#endif

  int dllDisc = GNSS_DLL_DISC(iE, qE, iL, qL);  // *2^30

  // volatile bo gcc wyrzuci zmienną i zepsuje wyniki!
  volatile int dllDiscFlt = GNSS_DLL_FLT(dllDisc); // /2^32
  
  // int dllDiscFlt = (int64_t)dllDisc * 2 / (21 * 1023);

#ifdef PRE_CARR_REM4_AND_DISC0_PROC
  GNSS_CODE_DISC(dllDiscFlt - prevDllDisc[channel]);
  prevDllDisc[channel] = dllDiscFlt;
#else

  //#warning GNSS_CODE_DISC OFF!!!!!!!
  GNSS_CODE_DISC(dllDiscFlt);
#endif

  *dllDisc_out = dllDiscFlt;

}


void loopFeedback(int channel, int iP, int qP, int iE, int qE, int iL, int qL)
{
  int pllDisc_out; 
  int dllDisc_out;
  
  loopFeedback_ex(channel, iP, qP, iE, qE, iL, qL, &pllDisc_out, &dllDisc_out);
  
}

void ChangeFiltersBand(Track *track, int pllBn, int dllBn)
{
    int pllC1, pllC2, dllC1, dllC2;
    track->pllBn = pllBn;
    track->dllBn = dllBn;
    FilterCoefs(track->Kpll, track->pllBn, track->sumtime_ms, &pllC1, &pllC2);
    FilterCoefs(track->Kdll, track->dllBn, track->sumtime_ms, &dllC1, &dllC2);
    GNSS_PLL_FLT_COEF(pllC1, pllC2);
    GNSS_DLL_FLT_COEF(dllC1, dllC2);
}


void FiltersChange_setup(FiltersChange *self, int startPllBn, int endPllBn, int startDllBn,  int endDllBn, int steps)
{
    self->steps = steps;
    if(!steps) return;

    self->pllDelta = (endPllBn - startPllBn) / (float)steps;
    self->pllBn = startPllBn;

    self->dllDelta = (endDllBn - startDllBn) / (float)steps;
    self->dllBn = startDllBn;
}


int FiltersChange_step(FiltersChange *self, Track *track)
{
    if (!self->steps)
        return 0;
    self->steps--;

    //(tu float, gdy int to zejdzie do 0)
    self->pllBn += self->pllDelta;
    self->dllBn += self->dllDelta;

    ChangeFiltersBand(track, .5f + self->pllBn, .5f + self->dllBn);

    //if(!state->steps)
    //{
    //    printdma("#comp PLL coefs : Bn=%d, C1=%d, C2=%d \n", track->pllBn, pllC1, pllC2);
    //    printdma("#comp DLL coefs : Bn=%d, C1=%d, C2=%d \n", track->dllBn, dllC1, dllC2);
    //}

    return 1;
}

//static volatile int dbg_read_accu_index = 0;

void readAccu(int *accu, int *lock)
{
    //GNSS_PTR->CTRL |= GNSS_CTRL_CODE_TICK_IE;
    //GNSS_PTR->CTRL = BitSetVal(GNSS_PTR->CTRL , 0,  0);

    //[zerowanie indeksu accu]

    //volatile uint32_t ch = GNSS_CHANN_GET();
    //GNSS_CHANN_SET(ch);

    //sleep_c(200);

    //if (dbg_read_accu_index%11 != 0)
    //    printdma("[dbg-bug-fund!]\n");
    volatile int accu_i = GNSS_ACCU_GET();
    //dbg_read_accu_index++;

    *lock = accu_i;
    for (int j = 0; j < 10; j++)
    {
        accu_i = GNSS_ACCU_GET();
        //dbg_read_accu_index++;
        accu[j] = accu_i;
    }

    //GNSS_PTR->CTRL = BitSetVal(GNSS_PTR->CTRL , 0,  1);
}

//void directDisc(int channel, int pllDisc, int dllDisc)
//{
//  GNSS_CHANN_SET(channel);
//
//  // TODO czy 2 wywołania mają efekty uboczne??
//  GNSS_CARR_DISC(-prevPllDisc[channel]);
//  GNSS_CARR_DISC(+pllDisc);
//  prevPllDisc[channel] = pllDisc;
//
//  GNSS_CODE_DISC(-prevDllDisc[channel]);
//  GNSS_CODE_DISC(+dllDisc);
//  prevDllDisc[channel] = dllDisc;
//}


#if(1)
int read_range_ex(int range_channel, Track *conf, uint64_t* tick_timestamp)
{

#if defined(DO_NOT_USE_FREE_UPDATE)
    uint32_t codeStep = (Q32 * conf->codeChipDiv) * conf->chipRate / conf->fsample;
    volatile uint32_t code_nco_pointer = GNSS_CODE_DISC(prevDllDisc[conf->channel]);
#else

    // to zadziała tylko gdy free_update!
    volatile uint32_t codeStep = GNSS_DLL_FLT(0);
    volatile uint32_t code_nco_pointer = GNSS_CODE_DISC(0);

#endif

    //uint32_t pptr = code_nco_pointer;
    code_nco_pointer -= codeStep;

    uint32_t code_pri_chip_addr;
    uint32_t code_pri_chip_pointer;

    volatile uint32_t rngRes;
    readRangeReg(range_channel, tick_timestamp, &rngRes);

    code_pri_chip_addr = rngRes & 0xFFFF;
    code_pri_chip_pointer = (rngRes >> 24) & 0xF;


    //todo wyeliminować float chip
    //int chipSamp = ((int64_t)(chip * code_pri_chip_pointer) + (int64_t) code_nco_pointer * chip / Q32) / codeChipDiv;

    int chipRateDivK = conf->chipRate / 1000;
    int chipSamp = ((int64_t)(conf->seqLen * code_pri_chip_pointer / chipRateDivK) + (int64_t) code_nco_pointer * conf->seqLen / chipRateDivK / Q32) / conf->codeChipDiv;

    //int seqSamp = code_pri_chip_addr * chip;
    int seqSamp = code_pri_chip_addr * conf->seqLen / chipRateDivK;
    seqSamp += chipSamp;

    //seqSamp = conf->seqLen - seqSamp;


    return seqSamp;

}


int read_range(int range_channel, Track *conf)
{
    uint64_t tick_timestamp;
    return read_range_ex(range_channel, conf, &tick_timestamp);
}
#endif

int64_t readRangeChipQ32(int range_channel, int chipDiv)
{
    return readRangeChipQ32ts(range_channel, chipDiv, 0);
}


int64_t readRangeChipQ32ts(int range_channel, int chipDiv, uint64_t* tick_timestamp)
{
    volatile uint32_t rngRes;
    readRangeReg(range_channel, tick_timestamp, &rngRes);
    return readRangeChipQ32raw(range_channel, chipDiv, rngRes);
}


int64_t readRangeChipQ32raw(int range_channel, int chipDiv, uint32_t rngRes)
{
#if defined(DO_NOT_USE_FREE_UPDATE)
    uint32_t codeStep = (Q32 * conf->codeChipDiv) * conf->chipRate / conf->fsample;
    volatile uint32_t code_nco_pointer = GNSS_CODE_DISC(prevDllDisc[conf->channel]);
#else

    // to zadziała tylko gdy free_update!
    volatile uint32_t code_nco_pointer = GNSS_CODE_DISC(0);
    volatile __maybe_unused uint32_t codeStep = GNSS_DLL_FLT(0);

#endif

    //volatile uint32_t prevNco = code_nco_pointer;
    //#warning temp dbg OFF
    code_nco_pointer -= codeStep;

    // nie cofam gdy zmienia sie sekwencja
    //if (code_pri_chip_addr == 0 && code_pri_chip_pointer == 0 && code_nco_pointer > prevNco)
    //    //code_nco_pointer += codeStep;
    //    return 52725019;  //TEMP!!!!!!!!


    return compChipQ32(range_channel, chipDiv, rngRes, code_nco_pointer); //, 0);

}


int64_t compChipQ32(int range_channel, int chipDiv, uint32_t rngRes, uint32_t code_nco_pointer) //, int codeNcoOvf)
{

    uint32_t code_pri_chip_addr;
    uint32_t code_pri_chip_pointer;

    code_pri_chip_addr = rngRes & 0xFFFF;
    code_pri_chip_pointer = (rngRes >> 24) & 0xF;

    //if (codeNcoOvf)
    if(0)
    {
        code_pri_chip_pointer += chipDiv - 1;
        code_pri_chip_pointer %= chipDiv;
    }

    // to zeby uniknac dzielenia
    //                          0,  1,  2,  3,  4,  5,  6,  7,  8
    static int chipDivExp2[] = {0,  0,  1, -1,  2, -1, -1, -1,  3};

    if (chipDiv > 8 || chipDivExp2[chipDiv] < 0)
        code_nco_pointer = (code_nco_pointer + ((int64_t) code_pri_chip_pointer << 32)) / chipDiv;
    else
        code_nco_pointer = (code_nco_pointer + ((int64_t) code_pri_chip_pointer << 32)) >> chipDivExp2[chipDiv];

    int64_t chipsQ32 = code_nco_pointer + (((int64_t) code_pri_chip_addr) << 32);

    return chipsQ32;

}



int64_t readRange_seqFractQ32(int range_channel, Track *conf)
{
    return readRangeChipQ32(range_channel, conf->codeChipDiv) / conf->pCodeLen;
}



// setupSerialAcq
void setup_serial_acq(Track *track, int first, int mixMode, int searchChipDiv, int blockStart)
{
    int channel = track->channel;
    GNSS_CHANN_SET(channel);

    setFreeAccu(channel, 0);

    // bocDiv przeskalowany do searchChipDiv
    //int deviceBocDiv = searchChipDiv * track->bocDiv / track->codeChipDiv;
    //int deviceChipDiv = searchChipDiv > deviceBocDiv ? searchChipDiv : deviceBocDiv;

    int isBoc = track->bocDiv > 0;

    int deviceChipDiv = isBoc ? (searchChipDiv > 2 ? searchChipDiv : 2) : searchChipDiv;
    int deviceBocDiv = isBoc ? deviceChipDiv / 2 : 0;

    int eplFreq = deviceChipDiv / searchChipDiv;


    uint32_t codeStep = (Q32 * deviceChipDiv) * track->chipRate / track->fsample;
    uint32_t carrStep = Q32 * abs(track->freq) / track->fsample;

    GNSS_CARR_FREQ(carrStep);

    //if(first)
    //    printdma("# mix=%d\n", mixMode);

    GNSS_CARR_SET(0, mixMode);

    GNSS_CARR_REM(0, 0);
    GNSS_CARR_REM(0, 0);

    GNSS_CARR_DISC(0);
    GNSS_CODE_DISC(0);

    GNSS_CODE_NCO_FREQ(codeStep, deviceBocDiv);
    GNSS_CODE_EPL_FREQ(eplFreq, deviceChipDiv);

    GNSS_PCODE_LEN(track->pCodeLen, 0, 0);
    GNSS_SCODE_LEN(track->sCodeLen);
    GNSS_PCODE_ADDR_SET(0, 0);
    GNSS_SCODE_ADDR_SET(0);

    if(first)
    {
        int cnt = track->pCodeLen / 32;
        if (track->pCodeLen % 32 > 0)
            cnt++;
        for (int i = 0; i < cnt; i++)
            GNSS_PCODE_WR(track->pCode[i]);

        cnt = track->sCodeLen / 32;
        if (track->sCodeLen % 32 > 0)
            cnt++;
        for (int i = 0; i < cnt; i++)
            GNSS_SCODE_WR(track->sCode[i]);

        // przygotowanie param. filtrów - na potem do fazy track
        track->sumtime_ms = 0.5f + (track->pCodeLen * 1000.0f) / track->chipRate;
        //printdma("# sumTime [ms] : %d \n", track->sumtime_ms);

        track->Kpll = (float)track->fsample / Q32 * track->codeChipDiv * 128 * 180 / Q16;
        FilterCoefs(track->Kpll, track->pllBn, track->sumtime_ms, &(track->pllC1), &(track->pllC2));
        //printdma("# PLL : Bn=%d, C1=%d, C2=%d \n", track->pllBn, track->pllC1, track->pllC2);

        track->Kdll = (float)track->fsample / Q32 / 2;
        FilterCoefs(track->Kdll, track->dllBn, track->sumtime_ms, &track->dllC1, &track->dllC2);
        //printdma("# DLL : Bn=%d, C1=%d, C2=%d \n", track->dllBn, track->dllC1, track->dllC2);

    }

    GNSS_PCODE_ADDR_SET(0, 0);
    GNSS_SCODE_ADDR_SET(0);

    if (track->sCodeLen > 0)
    {
        GNSS_SCODE_ADDR_SET(track->nh_idx % track->sCodeLen);
    }

    setFreeUpdate(channel, 0);
    //potrzebne VE VL
    setFreeAccuVel(channel, 1);

    if(!blockStart)
    {
        setFreeAccu(channel, 1);
    }
    //printdma("#F-AC=0x%08X\n", (unsigned)GNSS_FREE_ACCU_RD());
}



void switch_acq_to_track(Track *track, int pllFreeUpdate, int dllFreeUpdate)
{
    int channel = track->channel;

    uint32_t codeStep = (Q32 * track->codeChipDiv) * track->chipRate / track->fsample;

    GNSS_CHANN_SET(channel);

    GNSS_CODE_DISC(0);

    GNSS_CODE_NCO_FREQ(codeStep, track->bocDiv);
    GNSS_CODE_EPL_FREQ(1, track->codeChipDiv);

    GNSS_PLL_FLT_COEF(track->pllC1, track->pllC2);
    GNSS_PLL_FLT_RST();

    GNSS_DLL_FLT_COEF(track->dllC1, track->dllC2);
    GNSS_DLL_FLT_RST();

#ifndef DO_NOT_USE_FREE_UPDATE
    setFreeUpdateEx(channel, pllFreeUpdate, dllFreeUpdate);
#endif

    //BREAKPOINT();
    //volatile int64_t t1 = getPerfCounter();
    //printdma("\tSW: %d %d %d %d %d %d %d %d %d %d %d", channel, track->codeChipDiv,  track->chipRate, track->fsample, track->bocDiv, track->pllC1, track->pllC2, track->dllC1, track->dllC2, pllFreeUpdate, dllFreeUpdate);
    //printdma("\tSW");
    //volatile int64_t t2 = getPerfCounter();
    //printdma("\ttm = %d", (int) (t2 - t1));//14000
    //sleep_c(10000);
    //__asm__ volatile("nop;nop;nop;nop;nop;");
    //__asm__ volatile("nop;nop;nop;nop;nop;");
    //__asm__ volatile("nop;nop;nop;nop;nop;");
    //__asm__ volatile("nop;nop;nop;nop;nop;");
    //__asm__ volatile("nop;nop;nop;nop;nop;");
    //__asm__ volatile("nop;nop;nop;nop;nop;");
    //__asm__ volatile("nop;nop;nop;nop;nop;");
    //__asm__ volatile("nop;nop;nop;nop;nop;");
}


