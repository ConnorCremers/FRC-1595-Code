/*
 * Container.H
 *
 *  Created on: Feb 6, 2015
 *      Author: Connor
 */

#ifndef SRC_FUNCTIONS_H_
#define SRC_FUNCTIONS_H_

float checkMax(float liftAxis ,int JamesHit ,bool topSwitch ,bool botSwitch ,float potVal ,bool manual);
bool downButton(bool downButton, bool botSwitch);
bool upButton(bool upButton, bool topSwitch);

float adjust(float value); //brings the joystick input to desired output range
float adjustSmooth(float value, float pastValue); //brings joytsick input to desired output range and prevents jerkyness (?)

float rightPower(float throttle, float turn);
float leftPower(float throttle, float turn);

float integrate(float past, float current, float total, float setPoint, float constant, float wait, bool continuous);
float PIDify(float past, float current, float setPoint, float pC, float iE, float dC, float wait, bool continuous);

float NavXR(float throttle, float angleDif);
float NavXL(float throttle, float angleDif);

int toteLifter(bool toteChecker, int binCount);

#endif /* SRC_FUNCTIONS_H_ */
