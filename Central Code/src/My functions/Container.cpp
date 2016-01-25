/*
 * Functions.cpp
 *
 *  Created on: Feb 6, 2015
 *      Author: Connor
 */
#include "Functions.h"
#include "Math.h"


float checkMax(float liftAxis , int JamesHit ,bool topSwitch ,bool botSwitch ,float potVal ,bool manual){
	const float voltsPerLevel = .25;	//how much it has to turn per tote
	float height, liftPow;

	height = voltsPerLevel*JamesHit + 1;	//the plus one is because even at its limit it returns some things

	if(potVal < height){	//if it needs to lift more
		liftPow = 1;
	}
	if(potVal > height){	//if it is too high
		liftPow = -1;
	}
	if(fabs(height-potVal) < .1){		//if at right height
		liftPow = 0;
	}

	if(manual){					//if it has completed reaching the positing so mikey can control
		liftPow = liftAxis;
	}

	if(topSwitch && liftPow > 0){		//just so it can't break anything
		liftPow = 0;
	}
	if(botSwitch && liftPow < 0){
		liftPow = 0;
	}
	return(liftPow);
}

