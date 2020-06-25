/*
 * print-dma.h
 *
 *  Created on: Aug 10, 2018
 *      Author: mkrej
 */

#ifndef PRINT_DMA_H_
#define PRINT_DMA_H_

int printdma(char *fmt, ...);
int printdma_f(char *fmt, ...);


void printdma_wait();
void printdma_buf(char *buf, int buf_len);
void printdma_free();

void printdma_dbgSetBufLen(int len);
void printdma_dbgSteps();

int core_print(char *fmt, ...);

#endif /* PRINT_DMA_H_ */
