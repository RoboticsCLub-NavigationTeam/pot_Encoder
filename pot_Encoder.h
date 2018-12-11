/*
 * pot_Encoder.h
 * 
 * Created : 12/18/2018
 *  Author : nis_ane
 *   email : 073bex421.nischal@pcampus.edu.np
 */

#ifndef _POT_ENCODER_H_
#define _POT_ENCODER_H_

#include "utils/kalman.h"

extern Kalman_Filter gAngle_Filter;


void KF_Init();

#endif // !_POT_ENCODER_H