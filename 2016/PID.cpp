/*
 * Calculate.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: Connor
 */
#include "Functions.h"
float integrate(float current, float total, float setPoint, float constant, bool continuous){
	float curError;
	float curError = current - setPoint;
	float pastError = past - setPoint;
	if(continuous){
		if(curError > 180){
			curError = curError - 360;
		}
		else if(curError < -180){
			curError = curError+360;
		}
	}
	curError = curError * constant;
	total = total+curError*constant;
	return(total);
}

float PIDify(float past, float current, float setPoint, float pC, float iE, float dC, bool continuous){
	float curError
	,pE ,dE ,tE;

	curError = current-setPoint;
	if(continuous){
		if(curError > 180){
			curError = curError - 360;
		}
		else if(curError < -180){
			curError = curError + 360;
		}
		if(current - past > 180){
			current = current - 360;
		}
		if(current - past < -180){
			past = past + 360;
		}
	}

	pE = curError * pC;
	dE = (current-past)* dC;
	tE = pE + iE + dE;
	return(tE);
}

