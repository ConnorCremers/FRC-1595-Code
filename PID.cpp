/*
 * Calculate.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: Connor
 */
#include "Functions.h"
float integrate(float past, float current, float total, float setPoint, float constant, float wait, bool continuous){
	float tempError;
	float curError = current - setPoint;
	float pastError = past - setPoint;
	if(continuous){
		if(curError > 180){
			curError = curError - 360;;
		}
		if(pastError > 180){
			pastError = pastError - 360;;
		}
	}
	tempError = .5*(pastError+curError)*wait;
	tempError = tempError * constant;
	total = total+tempError;
	return(total);
}

float PIDify(float past, float current, float setPoint, float pC, float iE, float dC, float wait, bool continuous){
	float curError
	,pE ,dE ,tE;

	curError = current-setPoint;
	if(continuous){
		if(curError > 180){
			curError = curError - 360;;
		}
		if(current - past > 180){
			current = current - 360;
		}
		if(current - past < -180){
			past = past + 360;
		}
	}

	pE = curError * pC;
	dE = (current-past)/wait* dC;
	tE = pE + iE + dE;
	return(tE);
}

