/*
 * NavXdrive.cpp
 *
 *  Created on: Feb 7, 2015
 *      Author: Connor
 */
#include "Functions.h"



float NavXL(float throttle, float angleDif){
	float lPow;
	if(throttle>=0)	//it is different depending on direction
	{
		lPow = (throttle*(1-angleDif));
	}
	if(throttle<0)
	{
		lPow = (throttle*(1+angleDif));
	}
	return(lPow);
}

float NavXR(float throttle, float angleDif){
	float rPow;
	if(throttle>=0)
	{
		rPow = -throttle*(1+angleDif);
	}
	if(throttle<0)
	{
		rPow = -throttle*(1-angleDif);
	}
	return(-rPow);
}
