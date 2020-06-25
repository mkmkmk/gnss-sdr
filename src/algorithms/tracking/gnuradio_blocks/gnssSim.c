#if 0 || defined(GNSS_SIM)

#include "stdlib.h"
#include "assert.h"

#include "gnssSim.h"

#include "print-dma.h"

// tryb zapisu do log-a wejść do niektórych funkcji
#define GNSS_SIM_TRACE 0

#if GNSS_SIM_TRACE
#include <stdarg.h>
#endif


// ten tryb wyłącza symulowanie opóźnień takich jak są w procku
#define MODE_DELAYS_OFF (0)


// tmp !
#define SIM_PRI_CODE_LEN 32000
#define SIM_SEC_CODE_LEN 32000


static int32_t CHANNEL;

static uint32_t CARR_POINTER[SIM_CHANNEL_NUM];
static int32_t CARR_OV_COUNTER[SIM_CHANNEL_NUM];

static uint32_t CARR_CONF[SIM_CHANNEL_NUM];
static uint32_t CARR_FREQ[SIM_CHANNEL_NUM];
static uint32_t CARR_FREQ_BASE[SIM_CHANNEL_NUM];
//int32_t CARR_DISC[SIM_CHANNEL_NUM];

#define IQ_DELAY 2
static int32_t ResIDelayLine[SIM_CHANNEL_NUM][IQ_DELAY];
static int32_t ResQDelayLine[SIM_CHANNEL_NUM][IQ_DELAY];


static uint32_t CODE_NCO_POINTER[SIM_CHANNEL_NUM];
static uint32_t CODE_NCO_POINTER_PREV[SIM_CHANNEL_NUM];


static uint32_t CODE_NCO_FREQ[SIM_CHANNEL_NUM];
static uint32_t CODE_NCO_FREQ_BASE[SIM_CHANNEL_NUM];

static uint32_t CODE_PRI_CHIP_POINTER[SIM_CHANNEL_NUM];
static uint32_t CODE_PRI_MEM_ADDR[SIM_CHANNEL_NUM];
static int32_t CODE_PRI_MEM[SIM_CHANNEL_NUM][SIM_PRI_CODE_LEN];

static uint32_t CODE_PRI_CHIP_ADDR[SIM_CHANNEL_NUM];

static uint32_t CODE_SEC_CHIP_ADDR[SIM_CHANNEL_NUM];
static uint32_t CODE_SEC_MEM_ADDR[SIM_CHANNEL_NUM];
static int32_t CODE_SEC_MEM[SIM_CHANNEL_NUM][SIM_SEC_CODE_LEN];

static uint32_t CODE_EPL_POINTER[SIM_CHANNEL_NUM];
static uint32_t CODE_EPL_FREQ[SIM_CHANNEL_NUM];
static uint32_t CODE_CHIP_FREQ[SIM_CHANNEL_NUM];
static uint32_t CODE_PRI_LENGTH[SIM_CHANNEL_NUM];
static uint32_t CODE_SEC_LENGTH[SIM_CHANNEL_NUM];

static uint32_t CODE_BOC_POINTER[SIM_CHANNEL_NUM];
static uint32_t CODE_BOC_FREQ[SIM_CHANNEL_NUM];
static uint32_t CODE_BOC_STATE[SIM_CHANNEL_NUM];


static uint32_t CODE_E[SIM_CHANNEL_NUM];
static uint32_t CODE_P[SIM_CHANNEL_NUM];
static uint32_t CODE_L[SIM_CHANNEL_NUM];
static uint32_t CODE_VL[SIM_CHANNEL_NUM];
static uint32_t CODE_VE[SIM_CHANNEL_NUM];

static uint32_t bitMask32[32]=
  {
  0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000, 0x10000, 0x20000, 0x40000, 0x80000, 0x100000, 0x200000, 0x400000, 0x800000, 0x1000000, 0x2000000, 0x4000000, 0x8000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000
  };


static int32_t ACCU[SIM_CHANNEL_NUM][10];
static int32_t ACCU_SAVED[SIM_CHANNEL_NUM][10];

static uint32_t CARR_POINTER_SAVED[SIM_CHANNEL_NUM];
static int32_t CARR_OV_COUNTER_SAVED[SIM_CHANNEL_NUM];
static uint32_t CODE_NCO_POINTER_SAVED[SIM_CHANNEL_NUM];
static uint32_t CODE_PRI_CHIP_ADDR_SAVED[SIM_CHANNEL_NUM];
static uint32_t CODE_BOC_POINTER_SAVED[SIM_CHANNEL_NUM];
static uint32_t CODE_PRI_CHIP_POINTER_SAVED[SIM_CHANNEL_NUM];


static int32_t accu_state = 0;
static int32_t rng_state = 0;



static int64_t PLL_FLT_STATE[SIM_CHANNEL_NUM];
static uint32_t PLL_FLT_C1[SIM_CHANNEL_NUM];
static uint32_t PLL_FLT_C2[SIM_CHANNEL_NUM];

static int64_t DLL_FLT_STATE[SIM_CHANNEL_NUM];
static uint32_t DLL_FLT_C1[SIM_CHANNEL_NUM];
static uint32_t DLL_FLT_C2[SIM_CHANNEL_NUM];


#define CODE_DELAY 1

// FREE_ACCU => + 1
static int32_t CodeDelayLine[SIM_CHANNEL_NUM][CODE_DELAY + 1];


static int32_t ChipAddrDelayLine[SIM_CHANNEL_NUM][CODE_DELAY + 1];
static int32_t ChipPointerDelayLine[SIM_CHANNEL_NUM][CODE_DELAY + 1];


static int32_t BLOCK_CODE_GET;


static int32_t FREE_ACCU[SIM_CHANNEL_NUM];
static int32_t VEL_ACCU_EN[SIM_CHANNEL_NUM];

static int32_t AFE[SIM_CHANNEL_NUM];

static int32_t FREE_UPDATE_CARR[SIM_CHANNEL_NUM];
static int32_t FREE_UPDATE_CODE[SIM_CHANNEL_NUM];


static int32_t FREE_ACCU_TRIG_ON[SIM_CHANNEL_NUM];
static int32_t FREE_ACCU_TRIG_OFF[SIM_CHANNEL_NUM];
static int32_t FREE_ACCU_TRIG_ON_EN[SIM_CHANNEL_NUM];
static int32_t FREE_ACCU_TRIG_OFF_EN[SIM_CHANNEL_NUM];



static int _codeGetStartup[SIM_CHANNEL_NUM];

#ifdef PRE_CARR_REM4_AND_DISC0_PROC
static int prevPllDisc[SIM_CHANNEL_NUM];
static int prevDllDisc[SIM_CHANNEL_NUM];
#endif


static int64_t TICK_TIMESTAMP[SIM_CHANNEL_NUM];



static int32_t skip_samp_sim[SIM_CHANNEL_NUM];

static uint64_t prev_track_step_perf_counter;

static int RANGE_CHANNEL;

#define IS_PTICK(codes) ((codes & 0x80000000) == 0x80000000)
#define BitSet(arg,posn) ((arg) | (1L << (posn)))





//--------------  PERF_COUNTER --------------
static const uint64_t Q32 = 0x100000000ll;

pthread_mutex_t counter_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t range_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t trig_lock = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t tickf_lock = PTHREAD_MUTEX_INITIALIZER;


typedef struct
{
    uint32_t int_step;
    uint32_t fract_nco_step;

    uint32_t frac_nco;
    uint64_t counter;

} PfCounter;

void pf_counter_zero(PfCounter *state)
{
    state->int_step = 0;
    state->fract_nco_step = 0;
    state->frac_nco = 0;
    state->counter = 0;
}

void pf_counter_setup(PfCounter *state, int fsample)
{
    state->int_step = GNSS_SIM_PERF_COUNTER_FREQ / fsample;
    state->fract_nco_step = Q32 * GNSS_SIM_PERF_COUNTER_FREQ / fsample;
    state->frac_nco = 0;
    state->counter = 0;
}

void pf_counter_step(PfCounter *state)
{
    state->counter += state->int_step;
    uint32_t prev = state->fract_nco_step;
    state->frac_nco += state->fract_nco_step;
    if(state->frac_nco < prev)
    {
        state->counter++;
        // 54b counter, 10 lat w real time - raczej niepotrzebne
        if(state->counter == 0x40000000000000LL)
        {
            state->counter = 0;
            printdma("#{*** perf-counter OVF ***}");
        }

    }
}

uint64_t pf_counter_get(PfCounter *state)
{
    uint64_t res = state->counter;
    return res;
}


void ShiftRight(int32_t * buf,uint32_t bufLen);

int32_t GnssSimChannelsNumber()
{
    return SIM_CHANNEL_NUM;
}


static PfCounter pf_counter_mem;
static PfCounter *pf_counter = &pf_counter_mem;



void gnss_sim_init(int fsample)
{

#ifdef _WIN32
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
#endif

  CSR_CTRL_PTR->CPU_INFO_1 = SIM_CHANNEL_NUM << CPU_GNSS_ISE_SHIFT;

  pf_counter_setup(pf_counter, fsample);

  prev_track_step_perf_counter = 0;

  accu_state = 0;
  rng_state = 0;
  BLOCK_CODE_GET = 0;
  RANGE_CHANNEL = 0;

  for(int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
  {
    FREE_ACCU[ch] = 0;
    AFE[ch] = 0;
    VEL_ACCU_EN[ch] =0;
    FREE_UPDATE_CARR[ch] = 0;
    FREE_UPDATE_CODE[ch] = 0;

    FREE_ACCU_TRIG_ON[ch] = 0;
    FREE_ACCU_TRIG_OFF[ch] = 0;
    FREE_ACCU_TRIG_ON_EN[ch] = 0;
    FREE_ACCU_TRIG_OFF_EN[ch] = 0;

#ifdef PRE_CARR_REM4_AND_DISC0_PROC
    prevPllDisc[ch] = 0;
    prevDllDisc[ch] = 0;
#endif

    _codeGetStartup[ch] = 8;
    for(int j = 0; j < CODE_DELAY + 1; j++)
    {
      CodeDelayLine[ch][j] = 0;
      ChipPointerDelayLine[ch][j] = 0;
      ChipAddrDelayLine[ch][j] = 0;
    }

    for(int j = 0; j < 10; j++)
    {
      ACCU[ch][j] = 0;
      ACCU_SAVED[ch][j] = 0;
    }

    CARR_POINTER_SAVED[ch] = 0;
    CARR_OV_COUNTER_SAVED[ch] = 0;
    CODE_NCO_POINTER_SAVED[ch] = 0;

    CODE_PRI_CHIP_ADDR_SAVED[ch] = 0;
    CODE_BOC_POINTER_SAVED[ch] = 0;
    CODE_PRI_CHIP_POINTER_SAVED[ch] = 0;


    PLL_FLT_STATE[ch] = 0;
    PLL_FLT_C1[ch] = 0;
    PLL_FLT_C2[ch] = 0;

    DLL_FLT_STATE[ch] = 0;
    DLL_FLT_C1[ch] = 0;
    DLL_FLT_C2[ch] = 0;

    CARR_POINTER[ch] = 0;
    CARR_OV_COUNTER[ch] = 0;
    CARR_CONF[ch] = 0;
    CARR_FREQ[ch] = 0;

    for(int j = 0; j < IQ_DELAY; j++)
    {
      ResIDelayLine[ch][j] = 0;
      ResQDelayLine[ch][j] = 0;
    }

    CODE_NCO_POINTER[ch] = 0;
    CODE_NCO_POINTER_PREV[ch] = 0;
    CODE_NCO_FREQ[ch] = 0;

    CODE_PRI_CHIP_POINTER[ch] = 0;
    CODE_PRI_MEM_ADDR[ch] = 0;

    for(int j = 0; j < SIM_PRI_CODE_LEN; j++)
    {
      CODE_PRI_MEM[ch][j] = 0;
    }

    CODE_PRI_CHIP_ADDR[ch] = 0;

    CODE_SEC_CHIP_ADDR[ch] = 0;
    CODE_SEC_MEM_ADDR[ch] = 0;

    for(int j = 0; j < SIM_SEC_CODE_LEN; j++)
    {
      CODE_SEC_MEM[ch][j] = 0;
    }

    CODE_EPL_POINTER[ch] = 0;
    CODE_EPL_FREQ[ch] = 0;
    CODE_CHIP_FREQ[ch] = 0;
    CODE_PRI_LENGTH[ch] = 0;
    CODE_SEC_LENGTH[ch] = 0;

    CODE_BOC_POINTER[ch] = 0;
    CODE_BOC_FREQ[ch] = 0;
    CODE_BOC_STATE[ch] = 0;


    CARR_FREQ_BASE[ch] = 0;

    TICK_TIMESTAMP[ch] = 0;

    skip_samp_sim[ch] = 0;
  }

  printdma("-- RESET SIM STATE --\n");

}


void PrintGnssState()
{
  printdma("---------------- \n");
  printdma("CHANNEL = %d \n", CHANNEL);
  printdma("CARR_FREQ = %d (0x%x)\n", CARR_FREQ[CHANNEL], CARR_FREQ[CHANNEL]);
  printdma("CARR_POINTER = %d (0x%x)\n", CARR_POINTER[CHANNEL],CARR_POINTER[CHANNEL]);

  printdma("CODE_NCO_FREQ = %d (0x%x)\n", CODE_NCO_FREQ[CHANNEL],CODE_NCO_FREQ[CHANNEL]);
  printdma("CODE_NCO_POINTER = 0x%x\n", CODE_NCO_POINTER[CHANNEL]);
  printdma("CODE_PRI_CHIP_POINTER = %d \n", CODE_PRI_CHIP_POINTER[CHANNEL]);
  printdma("CODE_CHIP_FREQ = %d \n", CODE_CHIP_FREQ[CHANNEL]);
  printdma("CODE_PRI_CHIP_ADDR = %d \n", CODE_PRI_CHIP_ADDR[CHANNEL]);
  printdma("CODE_EPL_FREQ = %d \n", CODE_EPL_FREQ[CHANNEL]);
  printdma("CODE_EPL_POINTER = %d \n", CODE_EPL_POINTER[CHANNEL]);


  printdma("---------------- \n");

}


#if GNSS_SIM_TRACE

static int _vtrace_enabled = 1;

void vtrace_en(int enabled)
{
    _vtrace_enabled = enabled;
}

void vtrace(char *fmt, ...)
{
    va_list myargs;
    va_start(myargs, fmt);
    vprintf(fmt, myargs);
    va_end(myargs);
}
#else

void vtrace(char *fmt, ...){}
void vtrace_en(int enabled){}

#endif


void GNSS_STATUS_WR(uint32_t status)
{
    vtrace("|GNSS_STATUS_WR(%d)|", status);
}


void GNSS_CHANN_SET(int32_t ch)
{
    //maxtrace("|GNSS_CHANN_SET(%d)|", ch);
    CHANNEL = ch;
    accu_state = 0;
}

uint32_t GNSS_CHANN_GET(void)
{
    return CHANNEL;
}


void GNSS_CARR_FREQ(uint32_t carrNCOStep)
{
    vtrace("|GNSS_CARR_FREQ(0x%08x)|", carrNCOStep);
    CARR_FREQ[CHANNEL] = carrNCOStep;
    CARR_FREQ_BASE[CHANNEL] = carrNCOStep;
}


void GNSS_CARR_DISC_CH(int32_t disc, int ch, int block_update)
{
  // gnss.carr.disc

#ifndef PRE_CARR_REM4_AND_DISC0_PROC

    //CARR_FREQ[ch] = CARR_FREQ[ch] - CARR_FREQ_BASE[ch] + disc;
    //CARR_FREQ_BASE[ch] = disc;

    //todo czy taka blokada jest tez w procku??
    if(!block_update)
        CARR_FREQ[ch] = CARR_FREQ_BASE[ch] + disc;


#else
    CARR_FREQ[ch] = CARR_FREQ[ch] + disc;
#endif

}

// tego nie ma w procku ale nie ma też pośredniej zero i doplerów poniżej zera
static int freq_sign_ch(int ch)
{
    // ma być signed
    int sgn1 =  ((int32_t) CARR_FREQ[ch]) >= 0 ? +1 : -1;
    int sgn2 = (CARR_CONF[ch] & 0x02) ? -1 : 1;
    return sgn2 * sgn1;

}

uint32_t GNSS_CARR_DISC(int32_t disc)
{
    //vtrace("|GNSS_CARR_DISC(0x%08x)|", disc);
    if (!FREE_UPDATE_CARR[CHANNEL])
        GNSS_CARR_DISC_CH(disc, CHANNEL, FREE_UPDATE_CARR[CHANNEL]);
    return CARR_POINTER_SAVED[CHANNEL];
}

uint32_t gnss_dbg_get_carr_pointer_saved()
{
#warning  sign jest temp
    return CARR_POINTER_SAVED[CHANNEL] * freq_sign_ch(CHANNEL);
}

void GNSS_CARR_SET(uint32_t carrPointer, uint32_t carrConf)
{
    vtrace("|GNSS_CARR_SET(0x%08x, 0x%08x)|", carrPointer, carrConf);
    CARR_POINTER[CHANNEL] = carrPointer;
    CARR_CONF[CHANNEL] = carrConf;
}

static int _sigOvfToShow = 300;



int32_t GNSS_CARR_REM_CH(uint32_t iq, uint32_t mode, int ch)
{
    // tmp not implemented
    assert(mode == 0);

    static const int16_t sintab[] = { 1, 2, 2, 1, -1, -2, -2, -1 };
    static const int16_t costab[] = { 2, 1, -1, -2, -2, -1, 1, 2 };

    int16_t cos, sin;
    int16_t i, q;
    int32_t resQ, resI;
    int32_t resQ_out, resI_out;

    uint32_t prev_carr_ptr = CARR_POINTER[ch];
    CARR_POINTER[ch] = CARR_POINTER[ch] + CARR_FREQ[ch]; //+ CARR_DISC[CHANNEL];

    if(CARR_FREQ[ch] < 0x80000000)
    {
        if (CARR_POINTER[ch] < prev_carr_ptr)
        {
            CARR_OV_COUNTER[ch] += freq_sign_ch(ch);
            // 20b counter
            //CARR_OV_COUNTER[ch] &= 0xFFFFF;

        }
    }
    else
    {
        // tego nie ma w procku, ale w procku nie będzie nigdy pośredniej blisko zera,
        // to jest łata na symulację z cz. pośrednią zero
        // gdy carr LCO sam przepłynie na ujemne częstotliwości,
        // które się same zaimplementowały wartością CARR_FREQ bliską przepełnienia
        // TODO ale ta faza ogólnie powinna płynąć w ujemną stroną
        if (CARR_POINTER[ch] > prev_carr_ptr)
        {
            CARR_OV_COUNTER[ch] += freq_sign_ch(ch);
            // 20b counter
            //CARR_OV_COUNTER[ch] &= 0xFFFFF;
        }

    }

    // round(2*sin(0.5+0:7/8*2*pi))
    int32_t p = CARR_POINTER[ch] >> 29;
    sin = sintab[p];
    cos = costab[p];

    i = iq & 0xFFFF;
    q = (iq >> 16) & 0xFFFF;

// przełącznik próbki 16b / 4b
#if 1
    if (_sigOvfToShow && (i > 8 || i < -7 || q > 8 || q < -7))
    {
        printdma("gssSim: SIG-OVF! (%d, %d)\n", i, q);
        if(_sigOvfToShow == 1)
            printdma("gssSim: SIG-OVF warnings OFF! \n");
        _sigOvfToShow--;
    }
    // saturacja vs real?
    if(1)
    {
        if (i > 8)  i = 8;
        if (i < -7) i = -7;
        if (q > 8)  q = 8;
        if (q < -7) q = -7;
    }else
    {
        i = iq & 0xF;
        i = i & 0x8 ? i | 0xFFF0 : i;
        q = (iq >> 16) & 0xF;
        q = q & 0x8 ? q | 0xFFF0 : q;
    }
#endif



#ifndef PRE_CARR_REM4_AND_DISC0_PROC
    //  |  sig   |    lco     |  Conj(sig)  |  Conj(lco) |
    //  |--------|------------|-------------|------------|
    //  |  I+iQ  |  cos+isin  |    I-iQ     |  cos-isin  |

    // numeracja taka że 1 oznacza sprzężenie, łatwiej się zapamiętuje, taka memotechnika
    // ale niezgodna wstecz bo wcześniejsza opcja 0 jest teraz 1, do rozważenia czy wolimy zgodność wstecz czy wygodę na przyszłość
    if (CARR_CONF[ch] == 0)
    {
        // sig * lco
        resI = i * cos - q * sin;
        resQ = q * cos + i * sin;
    }
    else if (CARR_CONF[ch] == 1)
    {
        // sig * Conj(lco)
        resI = i * cos + q * sin;
        resQ = q * cos - i * sin;
    }
    else if (CARR_CONF[ch] == 2)
    {
        // Conj(sig) * lco
        resI = i * cos + q * sin;
        resQ = -q * cos + i * sin;
    }
    else // (CARR_CONF[ch]==3)
    {
        // Conj(sig) * Conj(lco)
        resI = i * cos - q * sin;
        resQ = -q * cos - i * sin;
    }
#else
    if (CARR_CONF[ch]==0)
    {
        // sig * Conj(lco)
        resI = i * cos + q * sin;
        resQ = q * cos - i * sin;
    }
    else
    {
        // Conj(sig) * Conj(lco)
        resI = i * cos - q * sin;
        resQ = -q * cos - i * sin;
    }
#endif


    resQ_out = ResQDelayLine[ch][IQ_DELAY - 1];
    ShiftRight(ResQDelayLine[ch], IQ_DELAY);
    ResQDelayLine[ch][0] = resQ;

    resI_out = ResIDelayLine[ch][IQ_DELAY - 1];
    ShiftRight(ResIDelayLine[ch], IQ_DELAY);
    ResIDelayLine[ch][0] = resI;

    if (MODE_DELAYS_OFF)
        return (resI & 0xFFFF) | ((resQ & 0xFFFF) << 16);

    return (resI_out & 0xFFFF) | ((resQ_out & 0xFFFF) << 16);
}


int32_t GNSS_CARR_REM(uint32_t iq, uint32_t mode)
{
    vtrace("|GNSS_CARR_REM(0x%08x, 0x%08x)|", iq, mode);
    return GNSS_CARR_REM_CH(iq, mode, CHANNEL);
}


void GNSS_CODE_NCO_FREQ(uint32_t codeNCOStep, uint32_t boc)
{
    vtrace("|GNSS_CODE_NCO_FREQ(0x%08x, 0x%08x)|", codeNCOStep, boc);
    // gnss.code.nco.freq
    CODE_NCO_FREQ_BASE[CHANNEL] = codeNCOStep;
    CODE_NCO_FREQ[CHANNEL] = codeNCOStep;
    CODE_BOC_FREQ[CHANNEL] = boc & 0xF;
}


void GNSS_CODE_EPL_FREQ(uint32_t eplFreq, uint32_t chipFreq)
{
    vtrace("|GNSS_CODE_EPL_FREQ(0x%08x, 0x%08x)|", eplFreq, chipFreq);
    // code.epl.freq
    CODE_EPL_FREQ[CHANNEL] = eplFreq & 0xF;
    CODE_CHIP_FREQ[CHANNEL] = chipFreq & 0xF;
}


void GNSS_PCODE_LEN(uint32_t codeLen, uint32_t coef, uint32_t scale)
{
    vtrace("|GNSS_PCODE_LEN(0x%08x, 0x%08x, 0x%08x)|", codeLen, coef, scale);
    // nss.pcode.len
    CODE_PRI_LENGTH[CHANNEL] = codeLen & 0xFFFF;
}


void GNSS_SCODE_LEN(uint32_t codeLen)
{
    vtrace("|GNSS_SCODE_LEN(0x%08x)|", codeLen);
    CODE_SEC_LENGTH[CHANNEL] = codeLen & 0x7F;
    // zero oznacza brak 2go kodu!
}


uint32_t GNSS_CODE_DISC_CH(int32_t disc, int ch, int block_update)
{
    // gnss.code.disc

#ifndef PRE_CARR_REM4_AND_DISC0_PROC

    //CODE_NCO_FREQ[ch] = CODE_NCO_FREQ[ch] - CODE_NCO_FREQ_BASE[ch] + disc;
    //CODE_NCO_FREQ_BASE[ch] = disc;

    //todo czy taka blokada jest tez w procku??
    if(!block_update)
        CODE_NCO_FREQ[ch] = CODE_NCO_FREQ_BASE[ch] + disc;

#else

    CODE_NCO_FREQ[ch] = CODE_NCO_FREQ[ch] + disc;

#endif

    return CODE_NCO_POINTER_SAVED[ch];
}


uint32_t GNSS_CODE_DISC(int32_t disc)
{
    return GNSS_CODE_DISC_CH(disc, CHANNEL, FREE_UPDATE_CODE[CHANNEL]);
}


void GNSS_PCODE_ADDR_SET(uint32_t adr, uint32_t codeNco)
{
    vtrace("|GNSS_PCODE_ADDR_SET(0x%08x, 0x%08x)|", adr, codeNco);
    // gnss.pcode.addr.set
    // #warning albo modulo albo >= w code_get()

    CODE_PRI_CHIP_ADDR[CHANNEL] = adr & 0xFFFF;
    CODE_PRI_MEM_ADDR[CHANNEL] = (adr & 0xFFFF) >> 5;
    CODE_BOC_POINTER[CHANNEL] = (adr >> 16) & 0xF; //GPR[rs][19 : 16]
    CODE_PRI_CHIP_POINTER[CHANNEL] = (adr >> 24) & 0xF; //% CODE_CHIP_FREQ[CHANNEL];

    CODE_NCO_POINTER[CHANNEL] = codeNco;
    CODE_NCO_POINTER_PREV[CHANNEL] = codeNco;
    CODE_EPL_POINTER[CHANNEL] = 0;
}


void GNSS_SCODE_ADDR_SET(uint32_t addr)
{
    vtrace("|GNSS_SCODE_ADDR_SET(0x%08x)|", addr);
    // gnss.scode.addr.set
    // todo o co w tym chodzi?
    //CODE_SEC_ACT_CHIP[CHANNEL] = CODE_SEC_MEM[CHANNEL][addr];
    //CODE_SEC_CHIP_ADDR[CHANNEL] = addr + 1;

    // mod. względem dok.
    CODE_SEC_CHIP_ADDR[CHANNEL] = addr & 0x7F;

}


void GNSS_PCODE_WR(uint32_t pcode)
{
    //vtrace("|GNSS_PCODE_WR(0x%08x)|", pcode);
    // gnss.pcode.wr
    CODE_PRI_MEM[CHANNEL][CODE_PRI_MEM_ADDR[CHANNEL]] = pcode;
    CODE_PRI_MEM_ADDR[CHANNEL] = CODE_PRI_MEM_ADDR[CHANNEL] + 1;
}


void GNSS_SCODE_WR(uint32_t scode)
{
    vtrace("|GNSS_SCODE_WR(0x%08x)|", scode);
    // gnss.scode.wr
    // 6543210
    CODE_SEC_MEM[CHANNEL][(CODE_SEC_CHIP_ADDR[CHANNEL] >> 5) & 0x3] = scode;
    CODE_SEC_CHIP_ADDR[CHANNEL] = CODE_SEC_CHIP_ADDR[CHANNEL] + 32;
}


void ShiftRight(int32_t * buf,uint32_t bufLen)
{
    int32_t i;
    for(i = bufLen-1; i>0; i--)
    {
        buf[i] = buf[i-1];
    }
}


void dbg_code_step()
{
    BLOCK_CODE_GET = 0;
    GNSS_CODE_GET();
    BLOCK_CODE_GET = 1;
}

uint32_t dbg_get_code_ptr()
{
    return CODE_NCO_POINTER[CHANNEL];
}



uint32_t dbg_get_code_addr()
{
    return CODE_PRI_CHIP_ADDR[CHANNEL];
}

void dbg_zero_code_ptr()
{
    //CODE_BOC_POINTER[CHANNEL] = 0;
    //CODE_PRI_CHIP_POINTER[CHANNEL] = 0;
//    CODE_NCO_POINTER[CHANNEL] = 0x40000000;
    //CODE_NCO_POINTER[CHANNEL] = 0;
    CODE_EPL_POINTER[CHANNEL] = 0;
}



#if(!MODE_DELAYS_OFF)
#define BLOCKED_CODE_DELAY_LINE_IDX(CH) (CODE_DELAY - 1 + FREE_ACCU[CH])
#else
#define BLOCKED_CODE_DELAY_LINE_IDX(CH) (0)
#endif


int32_t GNSS_CODE_GET_CH(int ch)
{

    if (BLOCK_CODE_GET)
    {
        return CodeDelayLine[ch][BLOCKED_CODE_DELAY_LINE_IDX(ch)];
    }

    // gnss.code.get
    uint32_t prevCodeNCO;
    uint32_t wordAddr, bitAddr, code;
    uint32_t codeWord;
    uint32_t res, resOut, pri_tick = 0, sec_tick = 0;

    prevCodeNCO = CODE_NCO_POINTER[ch];
    CODE_NCO_POINTER_PREV[ch] = CODE_NCO_POINTER[ch];
    CODE_NCO_POINTER[ch] = CODE_NCO_POINTER[ch] + CODE_NCO_FREQ[ch];

    if (prevCodeNCO > CODE_NCO_POINTER[ch])
    {
        CODE_PRI_CHIP_POINTER[ch] = CODE_PRI_CHIP_POINTER[ch] + 1;
        if (CODE_PRI_CHIP_POINTER[ch] >= CODE_CHIP_FREQ[ch])
        {
            CODE_PRI_CHIP_POINTER[ch] = 0;
            CODE_PRI_CHIP_ADDR[ch] = CODE_PRI_CHIP_ADDR[ch] + 1;
            if (CODE_PRI_CHIP_ADDR[ch] == CODE_PRI_LENGTH[ch])
            {
                CODE_PRI_CHIP_ADDR[ch] = 0;
                pri_tick = 1;

                pthread_mutex_lock(&counter_lock);
                TICK_TIMESTAMP[ch] = pf_counter_get(pf_counter);
                pthread_mutex_unlock(&counter_lock);

                CODE_SEC_CHIP_ADDR[ch] = CODE_SEC_CHIP_ADDR[ch] + 1;
                if (CODE_SEC_CHIP_ADDR[ch] >= CODE_SEC_LENGTH[ch])
                {
                    CODE_SEC_CHIP_ADDR[ch] = 0;
                    sec_tick = 1;
                }
            }
        }

        if (CODE_BOC_FREQ[ch])
        {
            CODE_BOC_POINTER[ch] += 1;

            if (CODE_BOC_POINTER[ch] >= CODE_BOC_FREQ[ch])
            {
                CODE_BOC_POINTER[ch] = 0;
                CODE_BOC_STATE[ch] ^= 1;
            }
        }

        CODE_EPL_POINTER[ch] = CODE_EPL_POINTER[ch] + 1;
        if (CODE_EPL_POINTER[ch] == CODE_EPL_FREQ[ch])
        {
            CODE_EPL_POINTER[ch] = 0;

            CODE_VL[ch] = CODE_L[ch];
            CODE_L[ch] = CODE_P[ch];
            CODE_P[ch] = CODE_E[ch];
            CODE_E[ch] = CODE_VE[ch];
        }
    }

    wordAddr = CODE_PRI_CHIP_ADDR[ch] / 32;
    bitAddr = CODE_PRI_CHIP_ADDR[ch] % 32;
    codeWord = CODE_PRI_MEM[ch][wordAddr];

    code = (codeWord & bitMask32[bitAddr]) != 0;

    if (CODE_SEC_LENGTH[ch] > 0)
    {
        wordAddr = CODE_SEC_CHIP_ADDR[ch] / 32;
        bitAddr = CODE_SEC_CHIP_ADDR[ch] % 32;
        codeWord = CODE_SEC_MEM[ch][wordAddr];

        // dod. negacja bo xor neguje dla 1 a mnożenie dla -1 [?]
        code ^= ((codeWord & bitMask32[bitAddr]) != 0) ^ 0x1;
    }

    if (CODE_BOC_FREQ[ch])
    {
        code ^= CODE_BOC_STATE[ch];
    }

    if (_codeGetStartup[ch] > 0)
    {
        _codeGetStartup[ch]--;
        code = 0;
    }

    CODE_VE[ch] = code ? 0x1 : 0xF;

    if (CODE_E[ch] == 0)
        CODE_E[ch] = CODE_VE[ch];
    if (CODE_P[ch] == 0)
        CODE_P[ch] = CODE_VE[ch];
    if (CODE_L[ch] == 0)
        CODE_L[ch] = CODE_VE[ch];
    if (CODE_VL[ch] == 0)
        CODE_VL[ch] = CODE_VE[ch];


    res = (CODE_P[ch]&0xF) |
        ((CODE_E[ch]&0xF) << 4) |
        ((CODE_L[ch]&0xF) << 8) |
        ((CODE_VE[ch]&0xF) << 12) |
        ((CODE_VL[ch]&0xF) << 16) |
        ((sec_tick&0x1) << 30) |
        ((pri_tick&0x1) << 31);

    //#warning DEBUG GENERATE ONES - ON/OFF
    //res = 0x11111 | ((pri_tick&0x1) << 31);

    resOut = CodeDelayLine[ch][CODE_DELAY - 1];
    ShiftRight(CodeDelayLine[ch], CODE_DELAY + FREE_ACCU[ch]);
    CodeDelayLine[ch][0] = res;

    ShiftRight(ChipAddrDelayLine[ch], CODE_DELAY + 1);
    ChipAddrDelayLine[ch][0] = CODE_PRI_CHIP_ADDR[ch];

    ShiftRight(ChipPointerDelayLine[ch], CODE_DELAY + 1);
    ChipPointerDelayLine[ch][0] = CODE_PRI_CHIP_POINTER[ch];

    if (MODE_DELAYS_OFF)
        return res;

    return resOut;

}


int32_t GNSS_CODE_GET()
{
    return GNSS_CODE_GET_CH(CHANNEL);
}


#ifdef CPU_CCNV1_A1
#define RNG_STATES_NUM 3
#else
#define RNG_STATES_NUM 4
#endif


uint32_t GNSS_CODE_RNG(int range_channel)
{
    //gnss.code.rng

    uint32_t res = 0;
    if(rng_state == 0)
    {

#ifndef CPU_CCNV1_A1
        if(range_channel >= 0)
#endif
        {
            pthread_mutex_lock(&range_lock);
            RANGE_CHANNEL = range_channel;
            pthread_mutex_unlock(&range_lock);
        }
        res =
              (CODE_PRI_CHIP_ADDR_SAVED[CHANNEL] & 0xFFFF) |
              ((CODE_BOC_POINTER_SAVED[CHANNEL] & 0xF) << 16) |
              ((CODE_PRI_CHIP_POINTER_SAVED[CHANNEL] & 0xF) << 24);

    }
    else if(rng_state == 1)
    {
    	pthread_mutex_lock(&counter_lock);
        res = TICK_TIMESTAMP[CHANNEL];
        pthread_mutex_unlock(&counter_lock);
    }
    else if(rng_state == 2)
    {
    	pthread_mutex_lock(&counter_lock);
        res = TICK_TIMESTAMP[CHANNEL] >> 32;
        pthread_mutex_unlock(&counter_lock);
    }
#ifndef CPU_CCNV1_A1
    else
    {
        //res = 888 + CHANNEL * 100000;
        res = CARR_OV_COUNTER_SAVED[CHANNEL];
    }
#endif

    rng_state++;
    if (rng_state > RNG_STATES_NUM - 1)
        rng_state = 0;

    return res;
}


int32_t GNSS_ACCU_GET_CH(int ch)
{
    int32_t res;
    if (accu_state == 0)
    {
        res = 0;
    }
    else
    {
        res = ACCU_SAVED[ch][accu_state - 1];
    }
    accu_state++;

    if (accu_state > 10)
        accu_state = 0;

    return res;
}


int32_t GNSS_ACCU_GET()
{
    return GNSS_ACCU_GET_CH(CHANNEL);
}


void GNSS_ACCU_ADD_CH(uint32_t samp, uint32_t code, int ch)
{

    int32_t icode, nextAccu;

    uint32_t pri_tick = (code >> 31) & 0x1;

    int32_t sigI = samp & 0xFFFF;
    sigI = sigI & 0x8000 ? sigI - 0x10000 : sigI;
    int32_t sigQ = (samp >> 16) & 0xFFFF;
    sigQ = sigQ & 0x8000 ? sigQ - 0x10000 : sigQ;

    int accuNum = VEL_ACCU_EN[ch] ? 5 : 3;

    for(int i = 0; i < accuNum; i++)
    {
        icode = (code >> (i * 4)) & 0xF;
        icode = icode & 0x8 ? icode - 0x10 : icode;

        nextAccu = ACCU[ch][i * 2 + 0] + sigI * icode;

        if (nextAccu >= 0x7FFFFF)
            ACCU[ch][i * 2 + 0] = 0x7FFFFF;
        else if (nextAccu <= -0x800000)
            ACCU[ch][i * 2 + 0] = -0x800000;
        else
            ACCU[ch][i * 2 + 0] = nextAccu;

        nextAccu = ACCU[ch][i * 2 + 1] + sigQ * icode;

        if (nextAccu >= 0x7FFFFF)
            ACCU[ch][i * 2 + 1] = 0x7FFFFF;
        else if (nextAccu <= -0x800000)
            ACCU[ch][i * 2 + 1] = -0x800000;
        else
            ACCU[ch][i * 2 + 1] = nextAccu;

    }

    if(pri_tick)
    {
        for(int i = 0; i < 10; i++)
        {
            ACCU_SAVED[ch][i] = ACCU[ch][i];
            ACCU[ch][i] = 0;
        }

		pthread_mutex_lock(&range_lock);
		int rng_ch = RANGE_CHANNEL;
		pthread_mutex_unlock(&range_lock);

        if(ch == rng_ch)
        {
            for (int ic = 0; ic < SIM_CHANNEL_NUM; ic++)
            {
                CARR_POINTER_SAVED[ic] = CARR_POINTER[ic];


                CARR_OV_COUNTER_SAVED[ic] = CARR_OV_COUNTER[ic];
                // 'licznik cykli jest zerowany po kazdym zatrzasnieciu range' (2019.06.15)
                //#warning bez zerowania CARR_OV_COUNTER!!!
                CARR_OV_COUNTER[ic] = 0;

#if defined(CPU_CCNV1_A1) || MODE_DELAYS_OFF
                CODE_NCO_POINTER_SAVED[ic] = CODE_NCO_POINTER[ic];
#else
                CODE_NCO_POINTER_SAVED[ic] = CODE_NCO_POINTER_PREV[ic];
#endif
                if (MODE_DELAYS_OFF)
                {
                    CODE_PRI_CHIP_ADDR_SAVED[ic] = CODE_PRI_CHIP_ADDR[ic];
                    CODE_PRI_CHIP_POINTER_SAVED[ic] = CODE_PRI_CHIP_POINTER[ic];
                }
                else
                {
                    CODE_PRI_CHIP_ADDR_SAVED[ic] = ChipAddrDelayLine[ic][CODE_DELAY];
                    CODE_PRI_CHIP_POINTER_SAVED[ic] = ChipPointerDelayLine[ic][CODE_DELAY];
                }

                CODE_BOC_POINTER_SAVED[ic] = CODE_BOC_POINTER[ic];

            }
        }
    }

}


void GNSS_ACCU_ADD(uint32_t valIn, uint32_t codeIn)
{
    //vtrace("|GNSS_ACCU_ADD(0x%08x,0x%08x)|", valIn, codeIn);
    //ta fun. jest bardzej userowa dlatego tu zerowanie rng_state
    rng_state = 0;
    GNSS_ACCU_ADD_CH(valIn, codeIn, CHANNEL);
}



void gnssSim_dbg_get_pointers(
        uint32_t *carr_pointer,
        uint32_t *code_nco_pointer,
        uint32_t *code_pri_chip_addr,
        uint32_t *code_boc_pointer,
        uint32_t *code_pri_chip_pointer
)
{
    *carr_pointer = CARR_POINTER_SAVED[CHANNEL];
    *code_nco_pointer = CODE_NCO_POINTER_SAVED[CHANNEL];
    *code_pri_chip_addr = CODE_PRI_CHIP_ADDR_SAVED[CHANNEL];
    *code_boc_pointer = CODE_BOC_POINTER_SAVED[CHANNEL];
    *code_pri_chip_pointer = CODE_PRI_CHIP_POINTER_SAVED[CHANNEL];
}

int gnssSim_dbg_getPCodeAdr(int ch)
{
    return CODE_PRI_CHIP_ADDR[ch];
}





int32_t dbg_last_pllDisc[SIM_CHANNEL_NUM];
int32_t dbg_last_dllDisc[SIM_CHANNEL_NUM];
int32_t dbg_last_pllDiscFlt[SIM_CHANNEL_NUM];
int32_t dbg_last_dllDiscFlt[SIM_CHANNEL_NUM];

void dbg_get_discs(
        int ch,
        int32_t *last_pllDisc,
        int32_t *last_pllDiscFilt,
        int32_t *last_dllDisc,
        int32_t *last_dllDiscFilt
    )
{

    *last_pllDisc = dbg_last_pllDisc[ch];
    *last_pllDiscFilt = dbg_last_pllDiscFlt[ch];

    *last_dllDisc = dbg_last_dllDisc[ch];
    *last_dllDiscFilt = dbg_last_dllDiscFlt[ch];

}

void dbg_get_accu(int ch, int32_t *accu)
{
    for (int i = 0; i < 10; ++i)
        accu[i] = ACCU_SAVED[ch][i];
}



int32_t GNSS_PLL_FLT_CH(int32_t inp, int ch);
int32_t GNSS_DLL_FLT_CH(int32_t inp, int ch);

//static int32_t filter(int32_t inp, uint32_t c1, uint32_t c2, int64_t *state);

static void freqFeedback(int ch)
{
  int pllDisc = GNSS_COST_DISC(ACCU_SAVED[ch][0], ACCU_SAVED[ch][1]);
  dbg_last_pllDisc[ch] = pllDisc;

  int pllDiscFlt = GNSS_PLL_FLT_CH(pllDisc, ch);

  dbg_last_pllDiscFlt[ch] = pllDiscFlt;
#ifdef PRE_CARR_REM4_AND_DISC0_PROC
  GNSS_CARR_DISC_CH(pllDiscFlt - prevPllDisc[ch], ch, 0);
  prevPllDisc[ch] = pllDiscFlt;
#else
  GNSS_CARR_DISC_CH(pllDiscFlt, ch, 0);
#endif
}




static void codeFeedback(int ch)
{
  int dllDisc = GNSS_DLL_DISC(ACCU_SAVED[ch][2], ACCU_SAVED[ch][3], ACCU_SAVED[ch][4], ACCU_SAVED[ch][5]);  // *2^30
  dbg_last_dllDisc[ch] = dllDisc;

  int dllDiscFlt = GNSS_DLL_FLT_CH(dllDisc, ch);

  dbg_last_dllDiscFlt[ch] = dllDiscFlt;
#ifdef PRE_CARR_REM4_AND_DISC0_PROC
  GNSS_CODE_DISC_CH(dllDiscFlt - prevDllDisc[ch], ch, 0);
  prevDllDisc[ch] = dllDiscFlt;
#else
  GNSS_CODE_DISC_CH(dllDiscFlt, ch, 0);
#endif
}



void gnssSim_skipSamp(int sampToSkip)
{
    if(1)
    {
        skip_samp_sim[CHANNEL] += sampToSkip;
        int32_t valCodes = GNSS_CODE_GET();
        //gdy tick to zdjęcie tick-a
        if(IS_PTICK(valCodes))
        {
            dbg_code_step();
            //niweluje dbg_code_step()
            skip_samp_sim[CHANNEL]++;
        }
    }
    else
    {
        // zamiast opóźniać generator, przyspiesza go
        // wszystko się odwraca, ale działa dalej bo ta sama funkcja jest
        // stosowana do skanowania i przestrajania na kod
        int toSkip = sampToSkip;
        while(toSkip > 0)
        {
            BLOCK_CODE_GET = 0;
            __attribute__((unused)) int32_t valCodes = GNSS_CODE_GET();
            BLOCK_CODE_GET = 1;
            toSkip--;
        }
    }
}


//#define TRACK_STEP_DELAY 1
//int32_t TrackStepDelayLine[SIM_CHANNEL_NUM][CODE_DELAY];

// dbg liczba próbek ominiętych przez GNSS_TRACK_STEP z powodu wyłączonego FREE_ACCU
static int dbg_no_free_accu_skipped[SIM_CHANNEL_NUM];

void GNSS_TRACK_STEP(int32_t valIn)
{
    vtrace_en(0);

    uint32_t lastTickF = 0;

    int32_t valCodes[SIM_CHANNEL_NUM];
    int32_t valCarrRems[SIM_CHANNEL_NUM];

    for(int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
    {
        if (AFE[ch] == 3)
            continue;

        if(skip_samp_sim[ch])
        {
            //skip_samp_sim[ch]--;
            continue;
        }

        if(!FREE_ACCU[ch])
        {
            //dbg_no_free_accu_skipped[ch]++;
            continue;
        }
        valCarrRems[ch] = GNSS_CARR_REM_CH(valIn, 0, ch); // remove carrier
        BLOCK_CODE_GET = 0;
        valCodes[ch] = GNSS_CODE_GET_CH(ch);// get spreading codes
        BLOCK_CODE_GET = 1;
    }

    for(int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
    {
        if (AFE[ch] == 3)
            continue;

        if(skip_samp_sim[ch])
        {
            skip_samp_sim[ch]--;
            continue;
        }
        if(!FREE_ACCU[ch])
        {
            dbg_no_free_accu_skipped[ch]++;
            continue;
        }

        GNSS_ACCU_ADD_CH(valCarrRems[ch],  valCodes[ch], ch);// accumulate (it will dump if PTICK is set in valCodes)

        if(IS_PTICK( valCodes[ch]))
        {
            if(FREE_UPDATE_CARR[ch])
            {
                freqFeedback(ch);
            }
            if(FREE_UPDATE_CODE[ch])
            {
                codeFeedback(ch);
            }
            lastTickF = BitSet(lastTickF, ch);
        }
    }

    GNSS_PTR->TICKF = lastTickF;

    if(PERFCNT_PTR->STATUS & PERFCNT_STAT_EN)
    {
        pthread_mutex_lock(&counter_lock);
        pf_counter_step(pf_counter);
        volatile uint64_t perf_counter = pf_counter_get(pf_counter);
        PERFCNT_PTR->CYCLE_LO = perf_counter;
        PERFCNT_PTR->CYCLE_HI = perf_counter >> 32;
        pthread_mutex_unlock(&counter_lock);

        pthread_mutex_lock(&trig_lock);
        for(int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
        {

            uint32_t prevPerf24 = prev_track_step_perf_counter & 0xFFFFFF;
            uint32_t perf24 = perf_counter & 0xFFFFFF;

            // gdy trigger jest pomiędzy kolejnymi odczytami
            if(FREE_ACCU_TRIG_ON_EN[ch])
            {
                if( (FREE_ACCU_TRIG_ON[ch] > prevPerf24 && FREE_ACCU_TRIG_ON[ch] <= perf24) ||
                    (perf24 < prevPerf24 && (FREE_ACCU_TRIG_ON[ch] > prevPerf24 || FREE_ACCU_TRIG_ON[ch] <= perf24)) )
                {
#ifdef CPU_CCNV1_A1
                    // przy ON są zerowane accu
                    for (int i = 0; i < 10; ++i)
                        ACCU[ch][i] = 0;
#endif
                    FREE_ACCU[ch] = 1;
                    FREE_ACCU_TRIG_ON_EN[ch] = 0;

                    if(0)
                        printdma("{s:%d}", dbg_no_free_accu_skipped[ch]);

                }
            }

            if(FREE_ACCU_TRIG_OFF_EN[ch])
            {
                if( (FREE_ACCU_TRIG_OFF[ch] > prevPerf24 && FREE_ACCU_TRIG_OFF[ch] <= perf24) ||
                    (perf24 < prevPerf24 && (FREE_ACCU_TRIG_OFF[ch] > prevPerf24 || FREE_ACCU_TRIG_OFF[ch] <= perf24)) )
                {
                    FREE_ACCU[ch] = 0;
                    dbg_no_free_accu_skipped[ch] = 0;
                    FREE_ACCU_TRIG_OFF_EN[ch] = 0;
                }
            }

        }
        pthread_mutex_unlock(&trig_lock);
        prev_track_step_perf_counter = perf_counter;
    }
    vtrace_en(1);
}


void GNSS_PLAY_STEP(int32_t valIn)
{
    uint32_t lastTickF = 0;

    int32_t valCodes[SIM_CHANNEL_NUM];
    int32_t valCarrRems[SIM_CHANNEL_NUM];

    for(int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
    {
        if (AFE[ch] != 3)
            continue;

        if (!FREE_ACCU[ch])
            continue;

        valCarrRems[ch] = GNSS_CARR_REM_CH(valIn, 0, ch); // remove carrier
        BLOCK_CODE_GET = 0;
        valCodes[ch] = GNSS_CODE_GET_CH(ch);// get spreading codes
        BLOCK_CODE_GET = 1;
    }

    for(int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
    {
        if (AFE[ch] != 3)
            continue;

        if(!FREE_ACCU[ch])
            continue;

        GNSS_ACCU_ADD_CH(valCarrRems[ch],  valCodes[ch], ch);// accumulate (it will dump if PTICK is set in valCodes)

        if(IS_PTICK( valCodes[ch]))
        {
            if(FREE_UPDATE_CARR[ch])
            {
                freqFeedback(ch);
            }
            if(FREE_UPDATE_CODE[ch])
            {
                codeFeedback(ch);
            }
            lastTickF = BitSet(lastTickF, ch);
        }
    }

    GNSS_PTR->TICKF = lastTickF;

}


void gnss_sim_perf_counter_lock()
{
	pthread_mutex_lock(&counter_lock);
}

void gnss_sim_perf_counter_unlock()
{
	pthread_mutex_unlock(&counter_lock);
}

void gnss_sim_tickf_lock()
{
    pthread_mutex_lock(&tickf_lock);
}

void gnss_sim_tickf_unlock()
{
    pthread_mutex_unlock(&tickf_lock);
}




void GNSS_FREE_ACCU_WR(int32_t conf, int32_t trigger)
{
    vtrace("|GNSS_FREE_ACCU_WR(0x%08x, 0x%08x)|", conf, trigger);
    if (trigger == 0)
    {
        int free_bit = 1;
        int vel_bit = 0x10000;
        //todo VEL_USE
        for (int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
        {
            FREE_ACCU[ch] = (conf & free_bit) != 0;
            VEL_ACCU_EN[ch] = (conf & vel_bit) != 0;
            free_bit <<= 1;
            vel_bit <<= 1;
        }

        FREE_ACCU_TRIG_ON_EN[CHANNEL] = 0;
        FREE_ACCU_TRIG_OFF_EN[CHANNEL] = 0;
    }
    else
    {
        pthread_mutex_lock(&counter_lock);
        // czemu tu był step??
        //pf_counter_step(pf_counter);
        volatile uint64_t perf_counter = pf_counter_get(pf_counter);
        pthread_mutex_unlock(&counter_lock);

        if (0 && (trigger & 0xFFFFFF) < (perf_counter & 0xFFFFFF))
        {
            printdma("#trig<counter\n");
        }

        pthread_mutex_lock(&trig_lock);
        if (conf == 0)
        {
            if(FREE_ACCU_TRIG_OFF_EN[CHANNEL])
            {
                printdma("\t#trig OFF ch %d EN\t", CHANNEL);
            }

            //trig off
            FREE_ACCU_TRIG_OFF_EN[CHANNEL] = 1;
            FREE_ACCU_TRIG_OFF[CHANNEL] = trigger & 0xFFFFFF;
        }
        else if (conf & (1 << CHANNEL))
        {
            if(FREE_ACCU_TRIG_ON_EN[CHANNEL])
            {
                printdma("\t#trig ON ch %d EN\t", CHANNEL);
            }
            //trig on
            FREE_ACCU_TRIG_ON_EN[CHANNEL] = 1;
            FREE_ACCU_TRIG_ON[CHANNEL] = trigger & 0xFFFFFF;
        }
        pthread_mutex_unlock(&trig_lock);
    }
}



int32_t GNSS_FREE_ACCU_RD()
{
    int free_bit = 1;
    int vel_bit = 0x10000;
    int res = 0;
    for (int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
    {
        if (FREE_ACCU[ch])
            res |= free_bit;
        if (VEL_ACCU_EN[ch])
            res |= vel_bit;
        free_bit <<= 1;
        vel_bit <<= 1;
    }
    return res;
}


void GNSS_FREE_UPDATE_WR(int32_t val)
{
    vtrace("|GNSS_FREE_UPDATE_WR(0x%08x)|", val);
    int carr_bit = 1;
    int code_bit = 0x10000;
    for (int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
    {
      FREE_UPDATE_CARR[ch] = (val & carr_bit) != 0;
      FREE_UPDATE_CODE[ch] = (val & code_bit) != 0;
      carr_bit <<= 1;
      code_bit <<= 1;
    }
}


uint32_t GNSS_FREE_UPDATE_RD()
{
    int carr_bit = 1;
    int code_bit = 0x10000;
    uint32_t res = 0;
    for (int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
    {
        if(FREE_UPDATE_CARR[ch])
            res |= carr_bit;
        if(FREE_UPDATE_CODE[ch])
            res |= code_bit;
        carr_bit <<= 1;
        code_bit <<= 1;
    }
    return res;
}


uint32_t GNSS_AFE_RD()
{
    int shift = 0;
    uint32_t res = 0;
    for (int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
    {
        res |= (AFE[ch] & 3) << shift;
        shift += 2;
    }
    return res;
}

void GNSS_AFE_WR(uint32_t val)
{
    vtrace("|GNSS_AFE_WR(0x%08x)|", val);
    for (int ch = 0; ch < SIM_CHANNEL_NUM; ch++)
    {
        AFE[ch] = val & 3;
        val >>= 2;
    }
}

// trunc(atan(q / i) * 180 / pi * 2^7 * 2^16)
static int32_t softCostDist(int32_t i, int32_t q)
{
    int64_t tmp;
    int32_t ret;
    int sign, cor, fract;

    fract = 23;
    cor = 0;

    sign = 1;
    if (q < 0) {
        sign *= -1;
        q      *= -1;
    }
    if (i < 0) {
        sign *= -1;
        i      *= -1;
    }

    if(i >= q){
        if (i == 0)
            return 0;
        tmp = ( (int64_t)q << fract ) / i;
    }
    else {
        if (q == 0)
            return 0;
        tmp = ( (int64_t)i << fract ) / q;
        cor = 1;
    }

    if ( tmp < ( 1 << (fract - 2) ) ){          //tmp <= 0.25
        tmp = 56 * tmp;                         //56 * tmp
    }
    else if ( tmp < ( 1 << (fract - 1) ) ){     //tmp <= 0.50
        tmp = 50 * tmp + ( 3 << (fract - 1) );  //50 * tmp + 1.5
    }
    else if ( tmp < ( 3 << (fract - 2) ) ){     //tmp <= 0.75
        tmp = 40 * tmp + (13 << (fract - 1) );  //40 * tmp + 6.5
    }
    else {                                      //tmp > 0.75
        tmp = 32 * tmp + (13 << fract);         //32 * tmp + 13
    }

    ret = (int32_t)tmp;

    if(cor == 1) {
        ret = (90 << fract) - ret;
    }

    ret *= sign;

    return ret;
}



int32_t GNSS_PLL_DISC(int32_t i, int32_t q)
{
    int32_t pi_2, cost;

    pi_2 = 90 << 23;
    cost = softCostDist(i, q);

    if(i >  0)           return cost;
    if(i <  0 && q >= 0) return cost + 2 * pi_2;
    if(i <  0 && q <  0) return cost - 2 * pi_2;
    if(i == 0 && q >  0) return pi_2;
    if(i == 0 && q <  0) return -1 * pi_2;

    return 0;
}


//todo gnss.pll.cost ?
int32_t GNSS_COST_DISC(int32_t i, int32_t q)
{
  return softCostDist(i,q);
}


//żeby nie było warning-a
#ifdef USE_FIND_MAX_BIT64_FUN_OR_BE_QUIET
static int findMaxBit64(int64_t value)
{
  uint64_t mask = 0x8000000000000000;

  if(value == 0) return 0;

  for (int i = 63; i > 0; i--)
  {
    if((value & mask) != 0) return i;
    mask >>= 1;
  }
  return 0;
}
#endif


int32_t GNSS_DLL_DISC(int32_t E_I, int32_t E_Q, int32_t L_I, int32_t L_Q)
{
  int64_t lPower, ePower, diff, sum;

  ePower = (int64_t) E_I * E_I + (int64_t) E_Q * E_Q;
  lPower = (int64_t) L_I * L_I + (int64_t) L_Q * L_Q;

  diff = ePower - lPower;
  sum = ePower + lPower;

  if (sum == 0)
  {
    return 0;
  }

  return (double)diff / sum * 0x40000000;

//  const int div = 11;
//  int mul = 62 + div - findMaxBit64(labs(diff));
//  if (mul > 30)
//    mul = 30;
//
//  if(sum >> div == 0)
//    return 0;
//
//  return ((diff << (mul - div)) / (sum >> div)) << (30 - mul);

}



void GNSS_PLL_FLT_COEF(uint32_t pllC1, uint32_t pllC2)
{
    PLL_FLT_C1[CHANNEL] = pllC1;
    PLL_FLT_C2[CHANNEL] = pllC2;
}

void GNSS_PLL_FLT_RST()
{
    PLL_FLT_STATE[CHANNEL] = 0;
}


static int32_t filter(int32_t inp, uint32_t c1, uint32_t c2, int64_t *state)
{
  int64_t TEMP_C1 = ((int64_t)inp * c1) >> 12;
  int64_t TEMP_C2 = ((int64_t)inp * c2) >> 12;
  int64_t res = (*state + TEMP_C1 + TEMP_C2) >> 20;
  *state += TEMP_C2;

  if (res > 0x80000000LL - 1)
  {
    res = 0x80000000 - 1;
  }

  if (res < -0x80000000LL)
  {
    res = -0x80000000;
  }

  return (int32_t)res;

}


int32_t GNSS_PLL_FLT_CH(int32_t inp, int ch)
{
    return filter(inp, PLL_FLT_C1[ch], PLL_FLT_C2[ch], PLL_FLT_STATE + ch);
}


int32_t GNSS_PLL_FLT(int32_t inp)
{
    if(!FREE_UPDATE_CARR[CHANNEL])
        return GNSS_PLL_FLT_CH(inp, CHANNEL);
    return CARR_FREQ[CHANNEL];
}


void GNSS_DLL_FLT_COEF(uint32_t dllC1, uint32_t dllC2)
{
    vtrace("|GNSS_DLL_FLT_COEF(0x%08x, 0x%08x)|", dllC1, dllC2);
    DLL_FLT_C1[CHANNEL] = dllC1;
    DLL_FLT_C2[CHANNEL] = dllC2;
}


void GNSS_DLL_FLT_RST()
{
    vtrace("|GNSS_DLL_FLT_RST()|");
    DLL_FLT_STATE[CHANNEL] = 0;
}


int32_t GNSS_DLL_FLT_CH(int32_t inp, int ch)
{
    return filter(inp, DLL_FLT_C1[ch], DLL_FLT_C2[ch], DLL_FLT_STATE + ch);
}


int32_t GNSS_DLL_FLT(int32_t inp)
{
    if(!FREE_UPDATE_CODE[CHANNEL])
        return GNSS_DLL_FLT_CH(inp, CHANNEL);
    return CODE_NCO_FREQ[CHANNEL];
}

// b. uproszczona impl.
uint32_t GNSS_STATUS_RD()
{
#ifdef CPU_CCNV1_A1
    return 0x00000001;
#else
    return 0x01000001;
#endif
}


void dbg_print_loop_state_header(FILE *file)
{
    char* header[] = { "idx", "CH", "IP", "QP", "IE", "QE", "IL", "QL", "IVE", "QVE", "IVL", "QVL", "pllDisc", "pllFt", "dllDisc", "dllFt" };
    int tp = 2 + 10 + 4;
    for (int i = 0; i < tp; i++)
        fprintf(file, "%s; ", header[i]);
    fprintf(file, "\n");
}

void dbg_print_loop_state(FILE *file, int channel, int64_t tickIdx)
{
    char *format = "%d;";
    char *format64 = "%lld;";
    fprintf(file, format64, tickIdx);
    fprintf(file, format, channel);

    int tp = 10;

    int32_t last_pllDisc;
    int32_t last_pllDiscFilt;
    int32_t last_dllDisc;
    int32_t last_dllDiscFilt;

    int accu[10];

    dbg_get_discs(
            channel,
            &last_pllDisc,
            &last_pllDiscFilt,
            &last_dllDisc,
            &last_dllDiscFilt
        );

    dbg_get_accu(channel, accu);

    for (int j = 0; j < tp; j++)
        fprintf(file, format, (int) accu[j]);
    fprintf(file, format, last_pllDisc);
    fprintf(file, format, last_pllDiscFilt);
    fprintf(file, format, last_dllDisc);
    fprintf(file, format, last_dllDiscFilt);

    fprintf(file, "\n");
}



#endif





