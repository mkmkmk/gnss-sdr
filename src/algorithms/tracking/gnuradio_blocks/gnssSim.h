/**
 * @file
 * @brief NaviSoC GNSS extension software simulator
 */

#ifndef __GNSS_SIM_H__
#define __GNSS_SIM_H__

#ifndef GNSS_SIM
#define GNSS_SIM
#endif

#ifdef GNSS_SIM

#include <stdint.h>
#include <stdio.h>
#include <pthread.h>

/**
 * Simulated performance counter frequency.
 * @return
 */
#define GNSS_SIM_PERF_COUNTER_FREQ (100000000)

/**
 * Number of simulated channels.
 * @return
 */
#define SIM_CHANNEL_NUM 16


/**
 * Returns number of simulated channels.
 * @return number of simulated channels
 */
int32_t GnssSimChannelsNumber();


/**
 * Initiates simulator state.
 */
void gnss_sim_init(int fsample);


/**
 * Prints GNSS simulator state.
 */
void PrintGnssState();


/**
 * Stores GNSS engine status register.
 * Dummy operation in simulator mode.
 * @param status value to store
 */
void  GNSS_STATUS_WR(uint32_t status);


/**
 * Computes Costas PLL discriminator value:
 *
 * \f[\tan^{-1}\left(\frac{Q_P}{I_P}\right)\f]
 * The return value is expressed in degrees (-180&deg;, +180&deg;)
 * multiplied by \f$2^{32}\f$ (Q8.23 format).
 *
 * @param i \f$I_P\f$ accumulator value
 * @param q \f$Q_P\f$ accumulator value
 * @return discriminator output
 */
int32_t GNSS_COST_DISC(int32_t i, int32_t q);

/**
 * Writes next primary code value of the current channel.
 *
 * Writes next 32 bits of primary code memory and increments CODE_PRI_MEM_ADDR by 32.
 *
 * @param pcode primary code bits
 */
void GNSS_PCODE_WR(uint32_t pcode);


/**
 * Writes next secondary code value of the current channel.
 *
 * Writes next 32 bits of secondary code memory and increments CODE_SEC_CHIP_ADDR by 32.
 *
 * @param scode secondary code bits
 */
void GNSS_SCODE_WR(uint32_t scode);


/**
 * Reads accumulator values saved on last tick.
 *
 * Reads accu[accu_state]. Every call increments accu_state.
 * First readout returns zero, next returns accu[0]. After 11 reads
 * accu_state variable is set to zero.
 *
 * Accumulators order:
 * \f[\left\{I_P, Q_P, I_E, Q_E, I_L, Q_L, I_{VE}, Q_{VE}, I_{VL}, Q_{VL} \right\}\f]
 *
 * @return next accumulator value
 */
int32_t GNSS_ACCU_GET();


/**
 * Selects GNSS channel.
 *
 * Also clears accu_state.
 * @param channNr channel index
 */
void GNSS_CHANN_SET(int32_t channNr);

/**
 * Returns current GNSS channel.
 * @return channel index
 */
uint32_t GNSS_CHANN_GET(void);

/**
 * Performs code removal and accumulation.
 *
 * When tick it saves accumulators (in ACCU_SAVED) and clears them.
 *
 * @param samp I/Q signal value (I - [3:0], Q - [19:16])
 * @param code next code values, see GNSS_CODE_GET()
 */
void GNSS_ACCU_ADD(uint32_t samp, uint32_t code);

/**
 * Sets PRN generator primary code address
 *
 * @param adr primary chip address - bits [15:0] and primary chip pointer - bits [27:24]
 * @param codeNco inside chip address (code generator NCO state)
 */
void GNSS_PCODE_ADDR_SET(uint32_t adr, uint32_t codeNco);


/**
 * Sets PRN generator secondary code address
 * @param addr code address
 */
void GNSS_SCODE_ADDR_SET(uint32_t addr);


/**
 * Sets primary code length
 *
 * @param codeLen primary code length
 */
void GNSS_PCODE_LEN(uint32_t codeLen, uint32_t coef, uint32_t scale);


/**
 * Sets secondary code length
 * @param codeLen
 */
void GNSS_SCODE_LEN(uint32_t codeLen);


/**
 * Sets code NCO frequency and BOC frequency (BOC chip divider)
 *
 * BOC frequency is a number of code NCO overflows to BOC switch.
 * BOC is not implemented.
 *
 * @param codeNCOStep code NCO frequency
 * @param boc BOC chip divider
 */
void  GNSS_CODE_NCO_FREQ(uint32_t codeNCOStep, uint32_t boc);


/**
 * Sets chip frequency (chip divider) and early-prompt-late change frequency.
 *
 * Early-prompt-late change frequency is a number of code NCO overflows to EPL switch.
 * Chip frequency is a number of code NCO overflows per chip.
 *
 * @param eplFreq early / prompt / late frequency
 * @param chipFreq chip frequency
 */
void  GNSS_CODE_EPL_FREQ(uint32_t eplFreq, uint32_t chipFreq);

/**
 * Computes next PRN generator value.
 *
 * Output format:
 * - bits [3:0] - CODE_P,
 * - bits [7:4] - CODE_E,
 * - bits [11:8] - CODE_L,
 * - bits [15:12] - CODE_VE,
 * - bits [19:16] - CODE_VL,
 * - bit [30] - SEC_TICK,
 * - bit [31] - PRI_TICK.
 * .
 *
 * @return next code values
 */
int32_t GNSS_CODE_GET();

int32_t GNSS_CODE_GET_CH(int ch);

/**
 * Writes GNSS accumulator mode configuration.
 *
 * Format:
 * - bits [15:0] - free accu mode for individual channels (0 - OFF, 1 - ON),
 * - bits [31:16] - very early and very late accumulators activity for individual channels (0 - OFF, 1 - ON).
 * .
 *
 * @param conf GNSS accumulator mode register configuration
 * @param trigger TODO
 */
void GNSS_FREE_ACCU_WR(int32_t conf, int32_t trigger);

/**
 * Reads GNSS accumulator mode configuration.
 *
 * Format - see GNSS_FREE_ACCU_WR().

 * @return configuration register value
 */
int32_t GNSS_FREE_ACCU_RD();

/**
 * Writes free running update configuration.
 *
 * Format:
 * - bits [15:0] - PLL free update for individual channels (0 - OFF, 1 - ON),
 * - bits [31:16] - DLL free update for individual channels (0 - OFF, 1 - ON).
 * .
 *
 * @param val configuration
 */
void GNSS_FREE_UPDATE_WR(int32_t val);

/**
 * Reads free running update configuration.
 *
 * Format - see GNSS_FREE_UPDATE_WR()
 *
 * @return configuration register value
 */
uint32_t GNSS_FREE_UPDATE_RD();


/**
 * Performs step in track step mode.
 *
 * Track step is equivalent of following operations sequence
 * performed for each free-mode enabled channel:
 * - GNSS_CARR_REM(),
 * - GNSS_CODE_GET(),
 * - GNSS_ACCU_ADD().
 * .
 *
 * @param valIn next signal I/Q sample (GNSS_ACCU_ADD() format)
 */
void GNSS_TRACK_STEP(int32_t valIn);

void GNSS_PLAY_STEP(int32_t valIn);


/**
 * Sets current channel carrier NCO frequency (NCO step)
 * @param carrNCOStep carrier NCO frequency
 */
void GNSS_CARR_FREQ(uint32_t carrNCOStep);


/**
 * Sets current channel carrier NCO pointer register value
 * and carrier mixing mode.
 *
 * Carrier mixing modes (carrConf):
 * - \a 0b00 - \f$ sig \cdot NCO \f$
 * - \a 0b01 - \f$ sig \cdot Conj(NCO) \f$
 * - \a 0b10 - \f$ Conj(sig) \cdot NCO \f$
 * - \a 0b11 - \f$ Conj(sig) \cdot Conj(NCO)\f$
 *
 * where:
 * \f[sig = I + jQ,\f]
 * \f[NCO = Re(NCO) + jIm(NCO),\f]
 * \f[Conj(sig) = I - jQ, \f]
 * \f[Conj(NCO) = Re(NCO) - jIm(NCO).\f]
 * @param carrPointer carrier NCO register value
 * @param carrConf carrier mixing mode
 */
void GNSS_CARR_SET(uint32_t carrPointer, uint32_t carrConf);


/**
 * Sets current channel PLL filter coefficients
 * @param pllC1 C1 coefficient
 * @param pllC2 C2 coefficient
 */
void GNSS_PLL_FLT_COEF(uint32_t pllC1, uint32_t pllC2);


/**
 * Resets current channel PLL filter state
 */
void GNSS_PLL_FLT_RST();


/**
 * Sets current channel DLL filter coefficients
 * @param dllC1 C1 coefficient
 * @param dllC2 C2 coefficient
 */

void GNSS_DLL_FLT_COEF(uint32_t dllC1, uint32_t dllC2);


/**
 * Resets DLL filter state
 */
void GNSS_DLL_FLT_RST();


/**
 * Computes PLL discriminator value for current channel:
 *
 * \f[atan2\left({Q_P}, {I_P}\right)\f]
 * The return value is expressed in degrees (-180&deg;, +180&deg;)
 * multiplied by \f$2^{32}\f$ (Q8.23 format).
 *
 * @param i \f$I_P\f$ accumulator value
 * @param q \f$Q_P\f$ accumulator value
 * @return discriminator output
 */
int32_t GNSS_PLL_DISC(int32_t i, int32_t q);


/**
 * Performs current channel carrier removal.
 *
 * Mixes signal value and current NCO according
 * to current mixing mode, see GNSS_CARR_SET().
 *
 * @param valIn signal I/Q sample (GNSS_ACCU_ADD() format)
 * @param aid not implemented
 * @return carrier removed signal sample
 */
int32_t GNSS_CARR_REM(uint32_t valIn, uint32_t aid);


/**
 * Computes current channel next PLL filter output
 * @param costDisc input value
 * @return output value
 */
int32_t GNSS_PLL_FLT(int32_t costDisc);


/**
 * Computes DLL discriminator value for current channel:
 *
 * \f[ \frac{(I_E^2 + Q_E^2) - (I_L^2 + Q_L^2)}{(I_E^2 + Q_E^2) + (I_L^2 + Q_L^2)}  \f]
 *
 * @param E_I \f$ I_E \f$ accumulator value
 * @param E_Q \f$ Q_E \f$ accumulator value
 * @param L_I \f$ I_L \f$ accumulator value
 * @param L_Q \f$ Q_L \f$ accumulator value
 * @return discriminator output
 */
int32_t GNSS_DLL_DISC(int32_t E_I, int32_t E_Q, int32_t L_I, int32_t L_Q);


/**
 * Computes current channel next DLL filter output
 * @param costDisc input value
 * @return output value
 */
int32_t GNSS_DLL_FLT(int32_t costDisc);


/**
 * Sets GNSS analog frontend engine configuration register (not implemented).
 * @param afe (not implemented)
 */
void GNSS_AFE_WR(uint32_t afe);


/**
 * Reads GNSS analog frontend engine configuration register.
 * @return 0 (not implemented)
 */
uint32_t GNSS_AFE_RD();


/**
 * Modifies carrier removal NCO frequency (according to filtered PLL discriminator value)
 * @param disc frequency change (e.g. filtered discriminator value)
 * @return CARR_POINTER_SAVED
 */
uint32_t GNSS_CARR_DISC(int32_t disc);


/**
 * Modifies code generator NCO frequency
 * (according to filtered DLL discriminator value).
 *
 * Returns
 *
 * @param disc NCO frequency change (e.g. filtered discriminator value)
 * @return
 */
uint32_t GNSS_CODE_DISC(int32_t disc);


/**
 * Returns code generator state or performance counter saved
 * on last tick of range channel used for
 * travel-time (and pseudo-range) computations.
 *
 * Instruction increments internal rng_state variable.
 * When rng_state = 0 instruction returns code generator state,
 * when rng_state = 1 instruction returns low 32b of performance counter,
 * when rng_state = 2 instruction returns high 32b of performance counter and sets rng_state to 0.
 *
 * Instruction sets range channel index.
 *
 * Code generator state output format:
 * - bits [15:00] - primary chip address,
 * - bits [19:16] - BOC pointer,
 * - bits [27:24] - primary chip pointer.
 *
 * @param range_channel range channel
 * @return code generator state or performance counter
 */
uint32_t GNSS_CODE_RNG(int range_channel);


uint32_t GNSS_STATUS_RD(void);

//debug func.
void dbg_get_pointers(
        uint32_t *carr_pointer,
        uint32_t *code_nco_pointer,
        uint32_t *code_pri_chip_addr,
        uint32_t *code_boc_pointer,
        uint32_t *code_pri_chip_pointer
);

//debug func.
int gnssSim_dbg_getPCodeAdr(int ch);

//debug func.
void dbg_get_discs(
        int ch,
        int32_t *last_pllDisc,
        int32_t *last_pllDiscFilt,
        int32_t *last_dllDisc,
        int32_t *last_dllDiscFilt
    );

void dbg_print_loop_state_header(FILE *file);

void dbg_print_loop_state(FILE *file, int channel, int64_t tickIdx);

//debug func.
void dbg_get_accu(int ch, int32_t *accu);

//debug func.
void dbg_code_step();

//debug func.
void dbg_zero_code_ptr();

//debug func.
uint32_t dbg_get_code_ptr();

//debug func.
uint32_t dbg_get_code_addr();


void gnssSim_skipSamp(int sampToSkip);

void gnss_sim_perf_counter_lock();

void gnss_sim_perf_counter_unlock();

void gnss_sim_tickf_lock();

void gnss_sim_tickf_unlock();


//---------------------------------------------------------
//                      udawane rejestry GNSS
//---------------------------------------------------------

#define CSR_STATUS_GET_CORE_ID(status) (2)


/** GNSS Controller Registers */
typedef struct
{
    uint32_t STATUS;    /*!< Status Register                                      */
    uint32_t COUNT;     /*!< Count Register                                       */
    uint32_t ADDRI;     /*!< I Array Address                                      */
    uint32_t ADDRQ;     /*!< Q Array Address                                      */
    uint32_t COUNTI;    /*!< Remaining I Count Register                           */
    uint32_t COUNTQ;    /*!< Remaining Q Count Register                           */
    uint32_t CTRL;      /*!< GNSS-ISE Control Register                            */
    uint32_t TICKF;     /*!< GNSS-ISE Code Tick Interrupt Flags Register          */
    uint32_t OVRF;      /*!< GNSS-ISE Code Tick Overrun Interrupt Flags Register  */
    uint32_t ERRF;      /*!< GNSS-ISE Code Error Interrupt Flags Register         */
    uint32_t IRQMAP;    /*!< Interrupt Mapping Register                           */
    uint32_t FIFOCNT;   /*!< FIFO Count Override (for diagnostics only)           */
    uint32_t FIFOWR;    /*!< FIFO Write Override (for diagnostics only)           */
} gnss_regs_t;


static gnss_regs_t foo_gnss_regs;
static volatile gnss_regs_t * const GNSS_PTR = &foo_gnss_regs;


/** GNSS Controller Status Register Flags */
enum
{
    GNSS_STAT_L1      = 1 << 0,  /*!< GNSS L1 Available                           */
    GNSS_STAT_L5      = 1 << 1,  /*!< GNSS L5 Available                           */
    GNSS_STAT_L2      = 1 << 2,  /*!< GNSS L2 Available                           */
    GNSS_STAT_L1_EN   = 1 << 4,  /*!< Enable L1 FIFO                              */
    GNSS_STAT_L5_EN   = 1 << 5,  /*!< Enable L5 FIFO                              */
    GNSS_STAT_L2_EN   = 1 << 6,  /*!< Enable L2 FIFO                              */
    GNSS_STAT_MODE    = 1 << 10, /*!< GNSS FIFO Mode                              */
    GNSS_STAT_PLAY    = 1 << 11, /*!< GNSS FIFO Play - dodałem                    */
    GNSS_STAT_BUSY    = 1 << 12, /*!< GNSS FIFO Busy                              */
    GNSS_STAT_START   = 1 << 13, /*!< GNSS FIFO Start                             */
    GNSS_STAT_ERR     = 1 << 16, /*!< GNSS FIFO Error                             */
    GNSS_STAT_OVF_I   = 1 << 17, /*!< GNSS FIFO I Overflow                        */
    GNSS_STAT_OVF_Q   = 1 << 18, /*!< GNSS FIFO Q Overflow                        */
    GNSS_STAT_ACQIE   = 1 << 20, /*!< Acquisition Interrupt Enable                */
    GNSS_STAT_ACQIF   = 1 << 21, /*!< Acquisition Interrupt Flag                  */
};



typedef struct
{
    uint32_t STATUS;            /*!< Status Register                        */
    uint32_t RET_PC;            /*!< Return Address                         */
    uint32_t EXCRSN;            /*!< Exception Reason                       */
    uint32_t BAD_ADDR;          /*!< Bad Address Register                   */
    uint32_t LL_ADDR;           /*!< Load Linked Address Register           */
    uint32_t ROM_UNLOCK;        /*!< ROM Unlock Register                    */
    uint32_t IRQ_HIST;          /*!< IRQ History                            */
    uint32_t CPU_INFO_0;        /*!< CPU Features 0                         */
    uint32_t CPU_INFO_1;        /*!< CPU Features 1                         */
    uint32_t DBG_BAUD;          /*!< Debug Baud Rate (mantisa and fraction) */
    uint32_t MEM_REMAP;         /*!< Memory Remap                           */
    uint32_t ICORE_IRQMAP;      /*!< Inter-Core IRQ Mapping                 */
    uint32_t ICORE_IRQTRIG;     /*!< Inter-Core IRQ Trigger                 */
    uint32_t ICORE_IRQF;        /*!< Inter-Core IRQ Flags                   */
    uint32_t SP_MIN;            /*!< Stack Protection Min Value             */
    uint32_t SP_MAX;            /*!< Stack Protection Max Value             */
    uint32_t IRQ_PRIOR[32];     /*!< Interrupt Priority                     */
    uint32_t IRQ_MASK;          /*!< Interrupt Mask                         */
    uint32_t CPU_TILE_ID;       /*!< Processor Tile Index                   */
    uint32_t CPU_IDCODE;        /*!< JTAG Format Processor IDCODE           */
} irq_regs_t;


static irq_regs_t foo_irq_regs;
static volatile irq_regs_t * const CSR_CTRL_PTR = &foo_irq_regs;


/** IRQ Controller Status Register flags */
enum
{
    CSR_STAT_EXC       = 0x0001,     /*!< Exception Support         */
    CSR_STAT_LLBIT     = 0x0004,     /*!< Load Linked Bit           */
    CSR_STAT_SPROTEN   = 0x0008,     /*!< Stack Protection Enable   */
    CSR_STAT_IR        = 0x0010,     /*!< Interrupt Request         */
    CSR_STAT_MR        = 0x0080,     /*!< Memory Remap              */
    CSR_STAT_CIEN      = 0x0100,     /*!< Current Interrupt Enable  */
    CSR_STAT_PIEN      = 0x0200,     /*!< Previous Interrupt Enable */
    CSR_STAT_OIEN      = 0x0400,     /*!< Old Interrupt Enable      */
    CSR_STAT_CMODE     = 0x0800,     /*!< Current Core Mode         */
    CSR_STAT_PMODE     = 0x1000,     /*!< Previous Core Mode        */
    CSR_STAT_OMODE     = 0x2000,     /*!< Old Core Mode             */
    CSR_STAT_IRQn      = 0x4000,     /*!< Pending Interrupt Number  */
    CSR_STAT_BD        = 0x80000,    /*!< Branch Delay              */
};


/** IRQ Controller CPU Info 1 Register bit offsets */
enum
{
    CPU_DCSIZE_SHIFT      = 1,    /*!< Data Cache Size Offset  */
    CPU_DCWAY_SHIFT       = 6,    /*!< Data Cache Ways Offset  */
    CPU_FPU_SHIFT         = 12,   /*!< FPU Offset              */
    CPU_GNSS_ISE_SHIFT    = 26,   /*!< GNSS Channels Offset    */
};

/** IRQ Controller CPU Info 1 Register masks */
enum
{
    CPU_DCSIZE_MASK      = 0x1F << CPU_DCSIZE_SHIFT,   /*!< Data Cache Size Mask      */
    CPU_DCWAY_MASK       = 0x03 << CPU_DCWAY_SHIFT,    /*!< Data Cache Ways Mask      */
    CPU_FPU_MASK         = 0x0F << CPU_FPU_SHIFT,      /*!< FPU Mask                  */
    CPU_GNSS_ISE_MASK    = 0x3F << CPU_GNSS_ISE_SHIFT, /*!< GNSS Channels Number Mask */
};


/** Performance Counter Registers */
typedef struct
{
    uint32_t STATUS;            /*!< Status Register                        */
    uint32_t CYCLE_LO;          /*!< Clock Cycle Counter LO                 */
    uint32_t CYCLE_HI;          /*!< Clock Cycle Counter HI                 */
} perfcnt_regs_t;


static perfcnt_regs_t foo_perfcnt_regs_t;
static volatile perfcnt_regs_t * const PERFCNT_PTR = &foo_perfcnt_regs_t;


/** Performance Counter Status Register flags */
enum
{
    PERFCNT_STAT_EN       = 0x0001,     /*!< Performance Counter Enable      */
};

//#define GNSS_STATUS_BUILD_RFAFE(rfafe) ((rfafe << GNSS_STAT_RFAFE_SHIFT) & GNSS_STAT_RFAFE_MASK)   /*!< Builds GNSS RF AFE   */
#define GNSS_STATUS_BUILD_RFAFE(rfafe) (0)

/** GNSS Controller Status RF AFE */
enum
{
    GNSS_RFAFE_L1     = 0x0,  /*!< GNSS L1 RF AFE              */
    GNSS_RFAFE_L5     = 0x1,  /*!< GNSS L5 RF AFE              */
    GNSS_RFAFE_L2     = 0x2,  /*!< GNSS L2 RF AFE              */
};

//typedef struct {
//    int core;
//} __lock_mutex_t;
//
//typedef __lock_mutex_t _LOCK_T;
//
//#define __LOCK_INIT(class,lock) class _LOCK_T lock __attribute__((unused)) __attribute__ ((section (".locks"))) = { 0 }
//
//#define __lock_acquire(lock) {}
//#define __lock_release(lock) {}



typedef pthread_mutex_t _LOCK_T;

#define __LOCK_INIT(class,lock) class _LOCK_T lock = PTHREAD_MUTEX_INITIALIZER;

#define __lock_acquire(lock) {  pthread_mutex_lock( &lock ); }

#define __lock_release(lock) {  pthread_mutex_unlock( &lock ); }

#define BREAKPOINT() {}

#define MEMORY_BARRIER() __asm__ __volatile__("": : :"memory")

static inline int syscalls_get_av_heap() { return -1; }

#else

static inline void gnss_sim_init()
{
}

static inline void gnss_sim_setup_pf_counter(int fsample)
{
}



static inline void gnss_sim_perf_counter_lock()
{
}

static inline void gnss_sim_perf_counter_unlock()
{
}

static inline void gnss_sim_tickf_lock()
{
}

static inline void gnss_sim_tickf_unlock()
{
}



#endif
#endif
