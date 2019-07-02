//
// Created by kjhertenberg on 16-5-19.
//

#ifndef ROBOTEAM_WORLD_CONSTANTSK_H
#define ROBOTEAM_WORLD_CONSTANTSK_H

namespace rtt {
// constant dimensions of the calculations
const int STATEINDEX = 6;
const int OBSERVATIONINDEX = 3;
// timerate
const float TIMEDIFF = 0.01;
//the different states the objects can be in
enum visState { NOT_VISIBLE, EXTRAPOLATED, VISIBLE };

// time after which objects disappear
const float DISAPPEARTIME=0.5/TIMEDIFF;//amount of ticks we wait for the object to disappear
const float EXTRAPOLATEDTIME=0.05/TIMEDIFF;// amount of ticks after we mark the object not visible but keep extrapolating it.

// amount of robots and balls per team that we keep track off
const int BOTCOUNT=16; //id 0-15
//used for checking convergence of K matrix
const float KMARGIN = 0.000001;
const int MAXCOMPARISONS = 100;

// constant variance estimates
// posVar: the distrust in the pos observation
// StateVar: the distrust in the initial state, which is the observation and such not defined
// RandVar: the distrust in the current state
// in the future data about them and us might be different so we made different variances.
// More trust in the model/state (higher Posvar compared to RandVar) leads to a smoother but slower signal
// More trust in the observation (higher Randvar compared to PosVar) leads to a noiser but faster signal
// Values should be bigger than 1
const float posVar = 5;
const float randVar = 10;
const float angleVar = 1;
const float posVar_ball = 1;
const float randVar_ball = 100;


const unsigned int INVALID_ID = 99;

}

#endif //ROBOTEAM_WORLD_CONSTANTSK_H
