#ifndef GOLDSEQ_L1_32_H
#define GOLDSEQ_L1_32_H

#include <stdint.h>
#include <stdlib.h>

/**
 * C/A code generation (1023 bits).
 *
 * @param n1 first tap of the phase selector (for the G2 register)
 * @param n2 second tap of the phase selector (for the G2 register)
 * @param *seq pointer to the buffer for the C/A code (32 byte buffer)
 */
void goldseq_L1_32(uint16_t n1, uint16_t n2, uint32_t *seq);

#endif

