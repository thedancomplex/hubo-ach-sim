#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>


#include <fcntl.h>
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"

#include "hubo.h"

// ach message type
typedef struct hubo huboOpen[1];

// ach channels
ach_channel_t chan_num;

// Timing info
#define NSEC_PER_SEC    1000000000


using namespace OpenRAVE;
using namespace std;

static inline void tsnorm(struct timespec *ts){

	while (ts->tv_nsec >= NSEC_PER_SEC) {
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}

int main( int argc, char ** argv) {

	printf("Start OpenHubo-Ach\n");

	int hflag = 0;
	int c;

	struct timespec t;
	int interval = 500000000; // 2hz (0.5 sec)
	//int interval = 10000000; // 100 hz (0.01 sec)



	huboOpen H;
	// open ach channel
	int r = ach_open(&chan_num, "hubo", NULL);
	assert( ACH_OK == r );
	size_t fs;
	r = ach_get( &chan_num, H, sizeof(H), &fs, NULL, ACH_O_LAST );
	assert( sizeof(H) == fs );
//	assert( ACH_OK == r );
/*
	while ((c = getopt (argc, argv, "h")) != -1) {
		switch (c) {
			case 'h':
				hflag = 1;
				scenefilename = argv[1];
				break;
			default:
				abort();
		}
	}

*/

	/* open openHubo */
	EnvironmentBasePtr penv;
	//string scenefilename = "openHubo/jaemiHubo.robot.xml";
	string scenefilename = "openHubo/jaemiHubo.robot.xml";
	RaveInitialize(true);
	//string scenefilename = "data/lab1.env.xml";
	//string viewername = "qtcoin";
	try {
	penv->Load(scenefilename);
	printf("Loaded scene: \n");
	}
	catch(exception e) {
		perror("load OpenRAVE ERR exiting");
		return -1;
	}
//	vector<RobotBasePtr> vrobots;
//	penv->GetRobots(vrobots);
//	RobotBasePtr probot = vrobots.at(0);
	
//	ModuleBasePtr pbasemanip = RaveCreateModule(penv,"basemanipulation");


	while(1) {


                // wait until next shot
                //clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);



		// get new hubo profile
		r = ach_get( &chan_num, H, sizeof(H), &fs, NULL, ACH_O_LAST );
		assert( sizeof(H) == fs );

		// do stuff here

		t.tv_nsec+=interval;
		tsnorm(&t);
	}

//	printf("%s",scenefilename);

}


