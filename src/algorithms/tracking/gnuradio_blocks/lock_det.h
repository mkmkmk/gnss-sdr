/*
 * lock_det.h
 *
 *  Created on: 16 wrz 2018
 *      Author: K. Marcinek + M. Krej
 */

#ifndef LOCK_DET_H_
#define LOCK_DET_H_


// K2 = K2_NUM / LOCK_DET_K2_DEN = 2^LOCK_DET_K2_NUM_BITS / LOCK_DET_K2_DEN

// default:
//#define LOCK_DET_K1_INV_BITS 10
//#define LOCK_DET_K2_NUM_BITS 1


#define LOCK_DET_K1_INV_BITS 10
#define LOCK_DET_K2_NUM_BITS 1


typedef struct
{
    int p_ifilt;
    int p_qfilt;

    int pes_lock_count;
    int pes_unlock_count;
    int opt_lock_count;
    int opt_unlock_count;

    int lock_pes;
    int lock_opt;

} lock_det;


void lock_det_init(lock_det *state);

int lock_det_next(lock_det *state, int ip, int qp);

void lock_det_dbg_print(lock_det *state);

int lock_det_cos2phi(lock_det *state);

#endif /* LOCK_DET_H_ */
