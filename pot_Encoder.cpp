/*
 * pot_Encoder.cpp
 * 
 * Created : 12/10/2018
 *  Author : nis_ane
 *   email : 073bex421.nischal@pcampus.edu.np
 */

#include "robot/pot_Encoder.h"
#include "utils/kalman.h"

struct Kalman_Vars gPot_Encoder;

Kalman_Filter gAngle_Filter(&gPot_Encoder);

void KF_Init()
{
    Mat state_model(2,2);
    state_model.at(0,0) = 1;
    state_model.at(0,1) = -1;
    state_model.at(1,0) = 0;
    state_model.at(1,1) = 1;

    Mat control_model(2,1);
    control_model.at(0,0) = 1;
    control_model.at(1,0) = 0;

    Mat obs_model(1,2);
    obs_model.at(0,0) = 1;
    obs_model.at(0,1) = 0;
    
    Mat eye(Mat::ident(2));
    
    Mat priori_error(2,2);
    priori_error.at(0,0) = 1000;
    priori_error.at(0,1) = 0;
    priori_error.at(1,0) = 0;
    priori_error.at(1,1) = 1000;
    
    Mat process_error(2,2);
    process_error.at(0,0) = 0.003;
    process_error.at(0,1) = 0;
    process_error.at(1,0) = 0;
    process_error.at(1,1) = 0.002;
    
    Mat measure_error(1,1);
    measure_error.at(0,0) = 100;
    
    gPot_Encoder.set_F(state_model);
    gPot_Encoder.set_B(control_model);
    gPot_Encoder.set_H(obs_model);
    gPot_Encoder.set_I(eye);
    gPot_Encoder.set_P(priori_error);
    gPot_Encoder.set_Q(process_error);
    gPot_Encoder.set_R(measure_error);
}

