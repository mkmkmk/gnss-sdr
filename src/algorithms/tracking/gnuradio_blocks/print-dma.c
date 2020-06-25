/*
 * print-dma.c
 *
 *  Created on: Aug 10, 2018
 *      Author: mkrej
 */

#ifndef GNSS_SIM
#include <ccproc.h>
#include <ccproc-amba.h>
#include <ccproc-amba-uart.h>
#include <ccproc-amba-dma.h>
#include <ccproc-perfcnt.h>
#include <ccproc-utils.h>
#include <ccproc-csr.h>
#include <sys/lock.h>
#include <board.h>
#endif

#include <stdio.h>
//#include <string.h>
//#include <time.h>
//#include <sys/time.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

#include "print-dma.h"


#define OVERFLOW_STRING_LEN 8
#define OVERFLOW_STRING ("~{OV!}\a\n")

#define DBG_PRINT_DMA_RECORD_STEPS (0)
#define DBG_STEPS_BUF_LEN 1000

#define DMA_CHANNEL (0)
#define DMA_BUF_LEN (2000)

static int _dma_buf_len = DMA_BUF_LEN;

#ifndef GNSS_SIM

static volatile char  _dma_buf[2][DMA_BUF_LEN];
// indeks bufora reload
static volatile int _dma_buff_idx = 0;
static int  _inited = 0;
static volatile amba_dma_channel_t *channel;

#endif


// to dla kompatybilności wstecz
#if BIN_TICKS_PRINT && !defined(LOG_AND_BIN_DATA_PRINT)
#define LOG_AND_BIN_DATA_PRINT (1)
#endif

#ifndef LOG_AND_BIN_DATA_PRINT
#define LOG_AND_BIN_DATA_PRINT (0)
#endif


#if DBG_PRINT_DMA_RECORD_STEPS
static int dbg_steps[DBG_STEPS_BUF_LEN];
static int dbg_step_idx = 0;
#endif


//#define VSNPRINTF_BUF_LEN 250
#define VSNPRINTF_BUF_LEN 2000


static inline void dbgRecordStep(int id)
{
#if DBG_PRINT_DMA_RECORD_STEPS
	//if(dbg_step_idx < DBG_STEPS_BUF_LEN)
    dbg_steps[dbg_step_idx] = id;
    dbg_step_idx++;
    if (dbg_step_idx == DBG_STEPS_BUF_LEN)
        dbg_step_idx = 0;
#endif
}


void printdma_dbgSetBufLen(int len)
{
    if (len > DMA_BUF_LEN)
    {
        printdma("# *** printdma_dbgSetBufLen() param error!!! \n");
        return;
    }
    _dma_buf_len = len;
}

//static volatile int __dbg_stop_breaks = 0;

#ifndef GNSS_SIM

static inline void ifNotInited()
{
	if(_inited) return;

	_inited = 1;

	channel = AMBA_DMA_DWN_CH_PTR(DMA_CHANNEL);
	AMBA_DMA_PTR->CONF |= DMA_CONF_DSEN;
	channel->PSELECT = AMBA_DMA_UART_PSELECT(STDIO_UART);

    channel->COUNTERREL = 0;
    channel->COUNTER = 0;

    // (start_dma)
    channel->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8 | DMA_CTRL_INC;
}


void printdma_dbgSteps()
{
    printdma("steps:\n");
#if DBG_PRINT_DMA_RECORD_STEPS
    for (int i = 0; i < dbg_step_idx; i++)
    printdma("%d, ", dbg_steps[i]);
#endif
    printdma("\n");

}

#endif


#if !defined(TURN_OFF_PRINT_DMA) && !defined(GNSS_SIM)

static inline void mycpy(volatile char *dst, volatile char *src, int len)
{
    for (int i = 0; i < len; ++i)
        dst[i] = src[i];
}


void checkstat()
{
    if (channel->STATUS & DMA_STAT_MERR)
    {
        BREAKPOINT();
    }
}


static inline void stop_dma()
{
    channel->CTRL = 0;
}


static inline void start_dma()
{
    channel->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8 | DMA_CTRL_INC;
}


void printdma_buf(char *buf, int buf_len)
{
    ifNotInited();
    //checkstat();

    volatile int stopped = 0;
    // mimo wielu zabezpieczeń bez tego coś chrzani się to na na FPGA
    // todo czy na CCNV1 też??
//#if 1 /*def BOARD_ML605*/
#ifdef BOARD_ML605
    if (channel->COUNTER == 1)
    {
        stop_dma();
        stopped = 1;
    }
#else
    if (channel->COUNTER == 1)
    {
        while (channel->COUNTER)
            __asm__ __volatile__("nop");
    }
#endif

    // *************** te linijki razem!
    // clear status
    __attribute__((unused)) volatile uint32_t status = channel->STATUS;
    MEMORY_BARRIER();
    volatile uint32_t counterrel = channel->COUNTERREL;
    channel->COUNTERREL = 0;
    // *********************************

    // gdyby przełączył się od wyczyszczenia STATUS
    if (channel->STATUS & DMA_STAT_MAR)
    {
        counterrel = 0;
        dbgRecordStep(6);
    }

    int ov = 0;

    // czy przełączył się na zapasowy bufor
    if (counterrel == 0)
    {
        dbgRecordStep(2);
        _dma_buff_idx = 1 - _dma_buff_idx;
    }

    if (counterrel + buf_len + OVERFLOW_STRING_LEN < _dma_buf_len)
    {
        dbgRecordStep(3);
        mycpy(_dma_buf[_dma_buff_idx] + counterrel, buf, buf_len);
        counterrel += buf_len;
    }
    else
    {
        ov = 1;
        dbgRecordStep(4);
        // brak miejsca nawet na overflow, wyrzucamy cały bufor do kosza
        if (counterrel + OVERFLOW_STRING_LEN >= _dma_buf_len)
        {
            dbgRecordStep(5);
            counterrel = 0;
        }
        mycpy(_dma_buf[_dma_buff_idx] + counterrel, OVERFLOW_STRING, OVERFLOW_STRING_LEN);
        counterrel += OVERFLOW_STRING_LEN;
        // BREAKPOINT();
    }

    channel->ADDRESSREL = (uint32_t)_dma_buf[_dma_buff_idx];
    MEMORY_BARRIER();
    channel->COUNTERREL = counterrel;


    if (stopped)
        start_dma();
}


#else

void printdma_buf(char *buf, int buf_len)
{
    //printf("%s", buf);
    //if(buf_len>1)buf[0]='~';
    write(1, buf, buf_len);
}

#endif

int printdma(char *fmt, ...);

void printdma_wait()
{
    //printdma("#br=%d\n", __dbg_stop_breaks);
#if !defined(TURN_OFF_PRINT_DMA) && !defined(GNSS_SIM)
    unsigned wdg = 0x100000;
    //while((channel->STATUS & (DMA_STAT_TCZ|DMA_STAT_RCZ)) == (DMA_STAT_TCZ|DMA_STAT_RCZ))
    //while((channel->STATUS & (DMA_STAT_TCZ|DMA_STAT_RCZ)))
	while(channel->COUNTER || channel->COUNTERREL)
	{
	    __asm__ __volatile__ ("nop");
	    if(!wdg--) break;
	}
#endif
}


int printdma(char *fmt, ...)
{
#ifdef TURN_OFF_PRINT
    return 0;
#endif

    static char buf[VSNPRINTF_BUF_LEN];
    va_list myargs;
    va_start(myargs, fmt);
    int toSend = vsnprintf(buf, VSNPRINTF_BUF_LEN, fmt, myargs);
    toSend = toSend > VSNPRINTF_BUF_LEN ? VSNPRINTF_BUF_LEN : toSend;
    va_end(myargs);

    // tryb przesyłania przez UART domyślnie binarnie,
    // zapis linii tekstu wymaga poprzedzenia tyldą
#if LOG_AND_BIN_DATA_PRINT
    int pop = 0;
    for (int i = 0; i < toSend; ++i)
    {
        if (buf[i] == '~')
            buf[i] = '*';
        if (buf[i] != '\n' && i < toSend - 1)
            continue;
        printdma_buf("~", 1);
        printdma_buf(buf + pop, i - pop + 1);
        pop = i + 1;
    }
#else
    printdma_buf(buf, toSend);
#endif

    return toSend;

}
#ifndef GNSS_SIM
__LOCK_INIT(static, uart_lock);
int core_print(char *fmt, ...) {
	static char buf[VSNPRINTF_BUF_LEN];
	va_list myargs;
	va_start(myargs, fmt);
	__lock_acquire(uart_lock);
	int toSend = sprintf(buf, "Core %u: ",
			(unsigned) CSR_STATUS_GET_CORE_ID(CSR_CTRL_PTR->STATUS));
	printdma_buf(buf, toSend);

	toSend = vsnprintf(buf, VSNPRINTF_BUF_LEN, fmt, myargs);
	toSend = toSend > VSNPRINTF_BUF_LEN ? VSNPRINTF_BUF_LEN : toSend;

	va_end(myargs);
	printdma_buf(buf, toSend);
	__lock_release(uart_lock);
	return toSend;
}

#endif
