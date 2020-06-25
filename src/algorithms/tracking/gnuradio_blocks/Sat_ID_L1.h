#ifndef SAT_ID_L1_H
#define SAT_ID_L1_H

#include <stdint.h>
#include <stdlib.h>

/**
 * Returns the number of first tap of the phase selector (for the G2 register).
 * @param sat Satellite ID number
 * @return n1 first tap
 */
uint16_t Sat_ID_L1_n1(uint16_t sat);

/**
 * Returns the number of second tap of the phase selector (for the G2 register).
 * @param sat Satellite ID number
 * @return n1 second tap
 */
uint16_t Sat_ID_L1_n2(uint16_t sat);

#endif

