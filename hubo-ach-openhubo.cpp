/** \example hubo-ach-openhubo.cpp
    \author Daniel M. Lofaro

    Loads OpenHUBO model and updates the joint values via what is avaliable on ach.
    If there is a colisiion a flag is posted.

    Usage:
    \verbatim
    hubo-ach-openhubo [-g] [-v] [-V]
    \endverbatim

    - \b -g - Shows the robot in the QTCoin viewer.
    - \b -v - verbose mode and prints out colision state only
    - \b -V = Extra Verbose mode prints out colision state and location of colision

    Example:
    \verbatim
    hubo-ach-openhubo  -g
    \endverbatim

 */
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>


/* ACH */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"



/* Hubo Const */
#include "hubo.h"

// ach message type
typedef struct hubo huboOpen[1];

// ach channels
ach_channel_t chan_num;

// Timing info
#define NSEC_PER_SEC    1000000000
//#define interval = 500000000; // 2hz (0.5 sec)
//#define interval = 10000000; // 100 hz (0.01 sec)

using namespace OpenRAVE;
using namespace std;


static inline void tsnorm(struct timespec *ts){

	while (ts->tv_nsec >= NSEC_PER_SEC) {
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}


void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AddViewer(viewer);

    // finally call the viewer's infinite loop (this is why a separate thread is needed)
    bool showgui = true;
    viewer->main(showgui);
}

int main(int argc, char ** argv)
{

	//int interval = 1000000000; // 1hz (1.0 sec)
	//int interval = 500000000; // 2hz (0.5 sec)
	int interval = 10000000; // 100 hz (0.01 sec)
	string viewername = "qtcoin";
	string scenefilename = "openHubo/jaemiHubo.robot.xml";
	RaveInitialize(true); // start openrave core
	EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
	
	int i = 1;
	bool vflag = false; // verbose mode flag
	bool Vflag = false; // extra verbose mode flag
	bool cflag = false; // self colision flag
	/* For Viewer */
	if(argc <= i ){
		printf("Loading Headless\n");	
	}
	while(argc > i) {
		if(strcmp(argv[i], "-g") == 0) {
			//RaveSetDebugLevel(Level_Debug);
			boost::thread thviewer(boost::bind(SetViewer,penv,viewername));
		}
		else {
			printf("Loading Headless\n");
		}

		
		if(strcmp(argv[i], "-v") == 0) {
			vflag = true;
			printf("Entering Verbose Mode");
		}
		else {
			vflag = false;
		}
		
		if(strcmp(argv[i], "-V") == 0) {
			Vflag = true;
			printf("Entering Extra Verbose Mode");
		}
		i++;
	}

	vector<dReal> vsetvalues;
	// parse the command line options
    
	// load the scene
	if( !penv->Load(scenefilename) ) {
		return 2;
	}


	/* timing */
	struct timespec t;
	
	/* hubo ACH Channel */
	huboOpen H;
	int r = ach_open(&chan_num, "hubo", NULL);
	size_t fs;


	/* read first set of data */
	r = ach_get( &chan_num, H, sizeof(H), &fs, NULL, ACH_O_LAST );
	assert( sizeof(H) == fs );


	int contactpoints = 0;
	bool runflag = true;
	while(runflag) {
	{



	// lock the environment to prevent data from changing
	EnvironmentMutex::scoped_lock lock(penv->GetMutex());

	//vector<KinBodyPtr> vbodies;
	vector<RobotBasePtr> vbodies;
	//penv->GetBodies(vbodies);
	penv->GetRobots(vbodies);
	// get the first body
	if( vbodies.size() == 0 ) {
		RAVELOG_ERROR("no bodies loaded\n");
		return -3;
	}

	//KinBodyPtr pbody = vbodies.at(0);
 	RobotBasePtr pbody = vbodies.at(0);
	vector<dReal> values;
	pbody->GetDOFValues(values);

	// set new values
	for(int i = 0; i < (int)vsetvalues.size() && i < (int)values.size(); ++i) {
		values[i] = vsetvalues[i];
	}

	
	pbody->Enable(true);
	//pbody->SetVisible(true);
	CollisionReportPtr report(new CollisionReport());
//	bool runflag = true;
//	while(runflag) {
	
		/* Wait until next shot */
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
		
		/* Get updated joint info here */
		r = ach_get( &chan_num, H, sizeof(H), &fs, NULL, ACH_O_LAST );
		assert( sizeof(H) == fs );


		/* set all joints from ach */
		for( int ii = 0; ii < (int)values.size() ; ii++ ) {
			values[ii] = H->joint[ii].ref;
		}



//		values[RSY] = -1.0;
//		values[REB] = 1.0;
        	pbody->SetDOFValues(values,true);



		penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
		if( pbody->CheckSelfCollision(report) ) {
			cflag = true;
		if(vflag | Vflag){
				RAVELOG_INFO("body not in collision\n");
			}
			if(Vflag) {   
			 	contactpoints = (int)report->contacts.size();
				stringstream ss;
				ss << "body in self-collision "
				<< (!!report->plink1 ? report->plink1->GetName() : "") << ":"
				<< (!!report->plink2 ? report->plink2->GetName() : "") << " at "
				<< contactpoints << "contacts" << endl;
				for(int i = 0; i < contactpoints; ++i) {
					CollisionReport::CONTACT& c = report->contacts[i];
					ss << "contact" << i << ": pos=("
					<< c.pos.x << ", " << c.pos.y << ", " << c.pos.z << "), norm=("
					<< c.norm.x << ", " << c.norm.y << ", " << c.norm.z << ")" << endl;
				}

			RAVELOG_INFOA(ss.str());
			}
		}
		else {
			cflag = false;
			if(vflag | Vflag) {
				RAVELOG_INFO("body not in collision\n");
			}
		}
        	// get the transformations of all the links
        	vector<Transform> vlinktransforms;
        	pbody->GetLinkTransformations(vlinktransforms);
		//boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		t.tv_nsec+=interval;
		tsnorm(&t);

//		runflag = false;
		//pbody->Enable(true);
		//pbody->SetVisible(true);
		pbody->SimulationStep(0.01);
		penv->StepSimulation(0.01);
	
	}
	}
	pause();
    RaveDestroy(); // destroy
    return contactpoints;
}
