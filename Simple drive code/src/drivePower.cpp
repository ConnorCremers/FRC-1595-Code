/*
 * leftPower.cpp
 *
 *  Created on: Feb 8, 2015
 *      Author: Connor
 */
#include "Functions.h"
#include "Math.h"
//this is Omair's code not mine
float leftPower(float throttle, float turn){

	float t_left, skimLeft, left, lPow;
	bool isSkimmingLeft;
	if(throttle != 0) {
		turn *= (1.1 * fabs(throttle));
	}
	t_left = throttle + turn;

	if (t_left > 1.0) {
		skimLeft = -((t_left - 1.0) * .7);
	}
	else if (t_left < -1.0) {
	    skimLeft = -((t_left + 1.0) * .7);
	}
	else {
		skimLeft = 0;
	}

	left = t_left + skimLeft;

	if (fabs(left) < 0.02) {
		left = 0;
	}


	lPow = left;


return(lPow);
}



float rightPower(float throttle, float turn){

float t_right, skimRight, right, rPow;

if(throttle != 0) {
	turn *= (1.1 * fabs(throttle));
}

t_right = throttle - turn;

if (t_right > 1.0) {
	skimRight = -((t_right - 1.0) * .7);
}
else if (t_right < -1.0) {
    skimRight = -((t_right + 1.0) * .7);
}
else {
	skimRight = 0;
}

right = t_right + skimRight;


if (fabs(right) < 0.02)
{
	right = 0;
}

rPow = right;
return(-rPow);

}


