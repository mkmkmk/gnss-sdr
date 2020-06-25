/*
 * lock_det.c
 *
 *  Created on: 16 wrz 2018
 *      Author: K. Marcinek + M. Krej
 */

#include <stdlib.h>
#include <stdio.h>


#include "lock_det.h"

#define OPT_LOCK_MAX 20
#define OPT_UNLOCK_MAX 2400
#define PES_LOCK_MAX 500
#define PES_UNLOCK_MAX 20


// K1 = 1/2^LOCK_DET_K1_INV_BITS
//#define LOCK_DET_K1_INV_BITS 5

//#warning lock-det mod K1
//#define LOCK_DET_K1_INV_BITS 10
//#define LOCK_DET_K1_INV_BITS 5


// K2 = K2_NUM / LOCK_DET_K2_DEN = 2^LOCK_DET_K2_NUM_BITS / LOCK_DET_K2_DEN
//#define LOCK_DET_K2_NUM_BITS 4
//#define LOCK_DET_K2_DEN 10


//#warning lock-det K2=3.2
//#define LOCK_DET_K2_NUM_BITS 5
//#define LOCK_DET_K2_DEN 10

// K2=2
//#define LOCK_DET_K2_NUM_BITS 1
//#define LOCK_DET_K2_DEN 1

//#define LOCK_DET_K2_NUM_BITS 2
//#define LOCK_DET_K2_DEN 1

//#define LOCK_DET_K2_NUM_BITS 1
//#define LOCK_DET_K2_DEN 1



#define LOCK_DET_K2_DEN 1


void lock_det_init(lock_det *state)
{
    state->p_ifilt = 0;
    state->p_qfilt = 0;

    state->pes_lock_count = 0;
    state->pes_unlock_count = 0;
    state->opt_lock_count = 0;
    state->opt_unlock_count = 0;

    state->lock_pes = 0;
    state->lock_opt = 0;

}

// round(2^16/100)
#define Q16_TO_PROC (655)

int lock_det_cos2phi(lock_det *state)
{
    int64_t ifilt = state->p_ifilt;
    int64_t qfilt = state->p_qfilt;


    int64_t num = ifilt * ifilt - qfilt * qfilt;
    int64_t den = ifilt * ifilt + qfilt * qfilt;

    if (den >> 16 == 0)
        return 100;

    return num / (den >> 16) / Q16_TO_PROC;
}


int lock_det_next(lock_det *state, int ip, int qp)
{

    //int ifilt = ((abs(ip) - state->p_ifilt) >> LOCK_DET_K1_INV_BITS) + state->p_ifilt;
    //int qfilt = ((abs(qp) - state->p_qfilt) >> LOCK_DET_K1_INV_BITS) + state->p_qfilt;
    //state->p_ifilt = ifilt;
    //state->p_qfilt = qfilt;

    // y[n] = y[n-1] + x[n] - y[n-1] / K_INV
    state->p_ifilt += abs(ip) - (state->p_ifilt >> LOCK_DET_K1_INV_BITS);
    state->p_qfilt += abs(qp) - (state->p_qfilt >> LOCK_DET_K1_INV_BITS);
    int ifilt = state->p_ifilt;
    int qfilt = state->p_qfilt;

    int sigOk = ifilt * LOCK_DET_K2_DEN > (qfilt << LOCK_DET_K2_NUM_BITS);

    // pessimistic phase lock calculate
    if (sigOk)
    {
        state->pes_lock_count++;
        state->pes_unlock_count = 0;
    }
    else
    {
        state->pes_unlock_count++;
        state->pes_lock_count = 0;
    }

    if (!state->lock_pes)
    {
        state->pes_unlock_count = 0;
        if (state->pes_lock_count > PES_LOCK_MAX)
        {
            state->lock_pes = 1;
            state->pes_unlock_count = 0;
            state->pes_lock_count = 0;
        }
    }
    else
    {
        state->pes_lock_count = 0;
        if (state->pes_unlock_count > PES_UNLOCK_MAX)
        {
            state->lock_pes = 0;
            state->pes_unlock_count = 0;
            state->pes_lock_count = 0;
        }
    }

    // optimistic phase lock calculate
    if (sigOk)
    {
        state->opt_lock_count++;
        state->opt_unlock_count = 0;
    }
    else
    {
        state->opt_unlock_count++;
        state->opt_lock_count = 0;
    }

    if (!state->lock_opt)
    {
        state->opt_unlock_count = 0;
        if (state->opt_lock_count > OPT_LOCK_MAX)
        {
            state->lock_opt = 1;
            state->opt_unlock_count = 0;
            state->opt_lock_count = 0;
        }
    }
    else
    {
        state->opt_lock_count = 0;
        if (state->opt_unlock_count > OPT_UNLOCK_MAX)
        {
            state->lock_opt = 0;
            state->opt_unlock_count = 0;
            state->opt_lock_count = 0;
        }
    }

    return state->lock_pes + 2 * state->lock_opt;

}


void lock_det_dbg_print(lock_det *state)
{
    //printf("ftI=%4d, ftQ=%4d, ftI/ftQ[%%]=%4d, pLc=%4d, pUn=%4d, oLc=%4d, oUn=%4d, ", state->p_ifilt, state->p_qfilt, abs(state->p_ifilt*100LL/state->p_qfilt), state->pes_lock_count, state->pes_unlock_count, state->opt_lock_count, state->opt_unlock_count);
    printf("%4d; %4d; %4d; %4d; %4d; %4d; %4d ", state->p_ifilt, state->p_qfilt, abs(state->p_ifilt*100LL/state->p_qfilt), state->pes_lock_count, state->pes_unlock_count, state->opt_lock_count, state->opt_unlock_count);

}




