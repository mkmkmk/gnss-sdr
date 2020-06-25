#ifndef __GNSS_TOOLS__H__
#define __GNSS_TOOLS__H__


#include <stdio.h>
#include <stdint.h>
#include <string.h>


#ifndef GNSS_SIM
#include <ccproc.h>
#include <ccproc-csr.h>
#include <ccproc-gnss-ise.h>
#include <board.h>

#else
#include "gnssSim.h"
#endif


//---------------------
#ifndef GNSS_SIM
#define PERF_COUNTER_FREQ CORE_FREQ
#else
#define PERF_COUNTER_FREQ GNSS_SIM_PERF_COUNTER_FREQ
#endif

//---------------------
#define __maybe_unused __attribute__((unused))
//---------------------
#define BitSet(arg,posn) ((arg) | (1L << (posn)))
#define BitClr(arg,posn) ((arg) & ~(1L << (posn)))
#define BitGet(arg,posn) (((arg) & (1L << (posn))) != 0)
//---------------------
#define IS_PTICK(codes) ((codes & 0x80000000) == 0x80000000)
//---------------------
#define MAX_CHAN 16

//---------------------
/// maska do zerowania GNSS_STATUS bez ruszania zarezerwowanych bitów
#define GNSS_STATUS_SAFE_ZERO_MASK (0xFFC8C08F)

#ifndef GNSS_SIM
//---------------------
// makro do rzeczywistego bezpiecznego ustawiania RF-AFE
#define GNSS_STATUS_SET_RFAFE(rfafe)                            \
    do {                                                        \
        GNSS_PTR->STATUS &= ~GNSS_STAT_RFAFE_MASK;              \
        GNSS_PTR->STATUS |= GNSS_STATUS_BUILD_RFAFE((rfafe));   \
    } while(0)

#else
#define GNSS_STATUS_SET_RFAFE(rfafe) {}
#endif

//---------------------
/**
 * Tracking channel configuration
 */
typedef struct
{

  /** channel index (GNSS-ISE) */
  int channel;

  /** sampling frequency [Hz] */
  int fsample;

  /** signal frequency [Hz] */
  int freq;

  /** chip frequency [Hz] */
  int  chipRate;

  /** primary code length */
  uint32_t pCodeLen;

  /** secondary code length */
  uint32_t sCodeLen;

  /** chip divider (a number of code NCO overflows per chip) */
  uint32_t codeChipDiv;

  /** BOC divider (a number of code NCO overflows to BOC switch) */
  uint32_t bocDiv;

  /** primary code buffer address (32 chips per word) */
  uint32_t *pCode;

  /** secondary code buffer address (32 chips per word) */
  uint32_t *sCode;

  /** number of samples per full code sequence */
  int seqLen;
  
  //int prevPllFilter;
  //int prevDllFilter;

  /** PLL filter bandwidth [Hz] */
  int pllBn;

  /** DLL filter bandwidth [Hz] */
  int dllBn;

  /** PLL loop gain */
  float Kpll;

  /** DLL loop gain */
  float Kdll;

  /** integration time (only for filters computation)*/
  int sumtime_ms;
  
  /** PLL loop coefficient */
  int pllC1;
  
  /** PLL loop coefficient */
  int pllC2;

  /** DLL loop coefficient */
  int dllC1;
  
  /** DLL loop coefficient */
  int dllC2;

  /** initial primary sequence shift [samples] */
  int seq_shift_samp; // [samples]
  
  /** initial secondary sequence shift*/
  int nh_idx; // 0..19

} Track;


/**
 * Configures GNSS-ISE channel for tracking.
 *
 * @param self track channel state
 * @param channel channel index
 * @param sig_freq signal frequency [Hz]
 * @param fsample sampling frequency [Hz]
 * @param chipRate chip frequency [Hz]
 * @param pCode primary code buffer address (32 chips per word)
 * @param pCodeLen primary code length
 * @param sCode secondary code buffer address (32 chips per word)
 * @param sCodeLen secondary code length
 * @param codeChipDiv chip divider (a number of code NCO overflows per chip)
 * @param bocDiv (a number of code NCO overflows to BOC switch)
 * @param mixMode carrier remove mixer mode
 * @param pllBn PLL filter bandwidth [Hz]
 * @param dllBn DLL filter bandwidth [Hz]
 * @param seq_shift_samp initial primary sequence shift [samples]
 */
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
        );



static inline int get_avail_chan_num()
{
    return (int)((CSR_CTRL_PTR->CPU_INFO_1 & CPU_GNSS_ISE_MASK) >> CPU_GNSS_ISE_SHIFT);
}


/**
 * Returns distance between chip index.
 *
 * @param a chip a index
 * @param b chip b index
 * @param codeLen code length
 * @return distance between chip index
 */
int moduloDiff(int a, int b, int codeLen);


/*
 * Bezpieczna dla CCNV1_A1 wersja GNSS_CODE_RNG()
 * zadziała dla CCNV1_A1 tylko gdy wywoływana zawsze jako
 * pierwsza zamiast GNSS_CODE_RNG()
 */
uint32_t gnss_code_rng_a1_safe(int32_t channel);

/**
 * Reads performance counter.
 *
 * @return performance counter value [CPU cycles]
 */
volatile uint64_t getPerfCounter();


void readRangeReg(int rangeChannel, volatile uint64_t *tickTm, volatile uint32_t *rangeReg);


void readRangeRegPh(int rangeChannel, volatile uint64_t *tickTm, volatile uint32_t *rangeReg, volatile uint32_t *carrCycles);


/**
 * Odczytuje czas tick-a (wartość performance counter-a)
 * @return czas tick-a [CPU cycles]
 */
volatile uint64_t getTickTime();

/**
 * Sleeps for a given number of CPU cycles.
 *
 * @param cycles CPU cycles
 */
void sleep_c(uint64_t cycles);


/**
 * Measures CPU frequency with the use of ADC clock.
 *
 * [mierzy częstotliwość CPU za pomocą zegara ADC]
 * One ADC connected free mode tracking channel is needed.
 * @param conf tracking channel configuration
 * @param irq_chan_ticks tick-ISR read tick counter
 * @param int4_lock tick-ISR lock object
 * @return ADC clock measured CPU clock frequency [Hz]
 */
int measClockFreq(Track *conf, volatile int *irq_chan_ticks/*, _LOCK_T *int4_lock*/);

// freqMeasTicks - liczba ticków pomiaru częstotliwości, duża wartość daje wiekszą dokładność
int measClockFreq_ex(Track *conf, volatile int *irq_chan_ticks, int freqMeasTicks);

int measClockFreqNew(Track *conf, int (*isrReadTicks)(int channel));

int measClockFreqNewEx(Track *conf, int (*isrReadTicks)(int channel), int freqMeasTicks);


/**
 * Stops free-accu for a given number of CPU cycles.
 * Applies hardware triggers. Maximum available precision.
 *
 * @param cpu_cycles pause time in CPU cycles
 * @param int4_lock tick-ISR lock object
 */
void pause_free_accu(int cpu_cycles/*, _LOCK_T *int4_lock*/);

/**
 *
 * @param cyclesToSkip
 * @param when 24b perf counter value when to start
 */
void pause_free_accu_at(int cyclesToSkip, uint64_t when);



/**
 * pauzuje wiele kanałów jednocześnie
 * @param cpu_cycles pause time in CPU cycles
 * @param channels_mask channels mask
 */
void pause_free_accu_multi(int cpu_cycles, uint32_t channels_mask);

void pause_free_accu_multi_at(int cpu_cycles, uint64_t when, uint32_t channels_mask);


/**
 * czy wywołania setFreeAccu() i setFreeAccuVel() (!)
 * są bezpieczne ze wzgledu na triggery
 * @return
 */
int isFreeAccuSetSafe();

/**
 * czy wywołania setFreeAccu() i setFreeAccuVel() (!)
 * są bezpieczne ze wzgledu na triggery, czas podawany jako parametr
 * @param time
 * @return
 */
int isFreeAccuSetSafeEx(uint64_t time);

/**
 * Sets free accumulator mode for selected channel.
 *
 * @param channel channel index
 * @param isOn 0-off, 1-on
 */
void setFreeAccu(int channel, int isOn);


/**
 * Sets very early and very late accumulators activity for selected channel.
 *
 * @param channel channel index
 * @param vel 0-off, 1-on
 */
void setFreeAccuVel(int channel, int vel);

/**
 * Reads free accumulator mode for selected channel.
 *
 * @param channel channel index
 * @return 0-off, 1-on
 */
volatile int getFreeAccu(int channel);


/**
 * Sets free update mode for selected channel.
 * @param channel channel index
 * @param vel 0-off, 1-on
 */
void setFreeUpdate(int channel, int isOn);

void setFreeUpdateEx(int channel, int pll, int dll);

void getFreeUpdate(int channel, int *pll, int *dll);

void setAfe(int chan, int val);

int getAfe(int chan);

/**
 * Enables GNSS interrupts.
 */
void enableGnssInterupts();


/**
 * Pauses interrupts with memory barrier
 */
void pause_interrupts();


/**
 *  Resumes interrupts with memory barrier
 */
void resume_interrupts();


/**
 *
 * @return 0 - interrupts are paused
 */
volatile int are_irq_enabled();


/**
 * docelowo obsolete, użyj track_setup()
 */
void setupChannel(int channel, Track *track, int mixMode);


/**
 * Sets code generator shift (primary code shift).
 * @param track tracking channel configuration
 * @param toShift primary code shift [samples]
 */
void setCodeShift(Track *track, int toShift);

//nieużywane?
void track_step(int sampI, int sampQ);
void readChannel(int channel, int *ptick_out, int* accu_out);


/**
 * Performs software loop feedback
 * @param channel GNSS-ISE channel index
 * @param iP \f$I_P\f$ accumulator value
 * @param qP \f$Q_P\f$ accumulator value
 * @param iE \f$I_E\f$ accumulator value
 * @param qE \f$Q_E\f$ accumulator value
 * @param iL \f$I_L\f$ accumulator value
 * @param qL \f$Q_L\f$ accumulator value
 * @param pllDisc_out current PLL filter output
 * @param dllDisc_out current DLL filter output
 */
void loopFeedback_ex(int channel, int iP, int qP, int iE, int qE, int iL, int qL, int *pllDisc_out, int *dllDisc_out);


/**
 * Performs software loop feedback
 * @param channel GNSS-ISE channel index
 * @param iP \f$I_P\f$ accumulator value
 * @param qP \f$Q_P\f$ accumulator value
 * @param iE \f$I_E\f$ accumulator value
 * @param qE \f$Q_E\f$ accumulator value
 * @param iL \f$I_L\f$ accumulator value
 * @param qL \f$Q_L\f$ accumulator value
 */
void loopFeedback(int channel, int iP, int qP, int iE, int qE, int iL, int qL);


//void directDisc(int channel, int pllDisc, int dllDisc);


/** Smooth filters bandwidth change (private) state */
typedef struct
{
    int steps;
    float pllDelta;
    float pllBn;
    float dllDelta;
    float dllBn;
} FiltersChange;

void FiltersChange_setup(FiltersChange *self, int startPllBn, int endPllBn, int startDllBn,  int endDllBn, int steps);
int FiltersChange_step(FiltersChange *self, Track *track);
void ChangeFiltersBand(Track *track, int pllBn, int dllBn);


/**
 * Reads accumulators and lock state.
 * Accumulators order:
 * \f[\left\{I_P, Q_P, I_E, Q_E, I_L, Q_L, I_{VE}, Q_{VE}, I_{VL}, Q_{VL} \right\}\f]
 *
 * @param accu output accumulators table
 * @param lock output lock state
 */
void readAccu(int *accu, int *lock);


/**
 * Computes second order loop filter coefficients
 * @param K loop gain
 * @param Bn loop bandwidth [Hz]
 * @param sumTimeMs integration time [ms]
 * @param c1 output C1 coefficient
 * @param c2 output C2 coefficient
 */
void FilterCoefs(float K, float Bn, int sumTimeMs, int *c1, int *c2);


/**
 * Reads pseudo-range from code generator state saved on tick on range channel
 *
 * @param range_channel next range channel
 * @param conf tracking channel config
 *
 * @return range channel tick code generator phase in samples
 */
int read_range(int range_channel, Track *conf);


/**
 * Reads pseudo-range from code generator state saved on tick on range channel
 *
 * @param range_channel next range channel
 * @param conf tracking channel config
 * @param tick_timestamp tick timestamp (performance counter, CPU cycles)
 *
 * @return range channel tick code generator phase in samples
 */
int read_range_ex(int range_channel, Track *conf, uint64_t* tick_timestamp);

/**
 * odczyt frakcji seqwencji w Q.32 zatrzaśniętego na ticku kanału range channel
 */
int64_t readRange_seqFractQ32(int range_channel, Track *conf);

/**
 * odczyt chipu z frakcją w Q.32 zatrzaśniętego na ticku kanału range channel
 * @param range_channel
 * @param chipDiv
 * @return
 */
int64_t readRangeChipQ32(int range_channel, int chipDiv);

int64_t readRangeChipQ32ts(int range_channel, int chipDiv, uint64_t* tick_timestamp);

int64_t readRangeChipQ32raw(int range_channel, int chipDiv, uint32_t rngRes);

int64_t compChipQ32(int range_channel, int chipDiv, uint32_t rngRes, uint32_t code_nco_pointer); //, int codeNcoOvf);

/**
 * Konfiguruje kanał do akwizycji szeregowej
 * @param track konfiguracja kanału
 * @param first true oznacza (długotrwające) wyliczanie parametrów filtrów
 * @param mixMode tryb mieszania
 * @param acq_chip_div
 * @param block_start - gdy 1 - nie włącza free accu
 */
void setup_serial_acq(Track *track, int first, int mixMode, int acq_chip_div, int block_start);


/**
 * Przełacza kanał z trybu akwizycji w tryb tracking-u
 * @param track
 */
void switch_acq_to_track(Track *track, int pllFreeUpdate, int dllFreeUpdate);



#endif
