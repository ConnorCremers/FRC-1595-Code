/*
 * totesMegoats.cpp
 *
 *  Created on: Feb 9, 2015
 *      Author: Connor
 */
#include "Functions.h"

int toteLifter(bool toteChecker, int binCount){
	int lift;
	if(toteChecker && binCount <6){	//if it needs to go up and down
		lift = 2;
	}
	else if(binCount == 6){	//if it needs to go down
		lift = 0;
	}
	else{lift = 1;}	//defaults to up

	return(lift);

}
