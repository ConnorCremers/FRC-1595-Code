/*
 * Functions.h
 *
 *  Created on: Jan 24, 2016
 *      Author: Connor
 */

#ifndef SRC_FUNCTIONS_H_
#define SRC_FUNCTIONS_H_


float adjust(float value);
float leftPower(float throttle, float turn);
float rightPower(float throttle, float turn);
float integrate(float current, float total, float setPoint, float constant, bool continuous);
float PIDify(float past, float current, float setPoint, float pC, float iE, float dC, bool continuous);
#endif /* SRC_FUNCTIONS_H_ */
