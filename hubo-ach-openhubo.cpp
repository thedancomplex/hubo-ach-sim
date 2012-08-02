/** \example orcollision.cpp
    \author Rosen Diankov

    Load a robot into the openrave environment, set it at [joint values] and check for self
    collisions. Returns number of contact points.

    Usage:
    \verbatim
    orcollision [--list] [--checker checker_name] [--joints #values [values]] body_model
    \endverbatim

    - \b --list - List all the loadable interfaces (ie, collision checkers).
    - \b --checker - name Load a different collision checker instead of the default one.
    - <b>--joints \#values [values]</b> - Set the robot to specific joint values

    Example:
    \verbatim
    orcollision --checker ode robots/barrettwam.robot.xml
    \endverbatim

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "../hubo-ACH/hubo.h"

using namespace OpenRAVE;
using namespace std;

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

	string viewername = "qtcoin";
	string scenefilename = "openHubo/jaemiHubo.robot.xml";
	RaveInitialize(true); // start openrave core
	EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
	/* For Viewer */
	RaveSetDebugLevel(Level_Debug);
	boost::thread thviewer(boost::bind(SetViewer,penv,viewername));
	vector<dReal> vsetvalues;
	// parse the command line options
	int i = 1;
    
	// load the scene
	if( !penv->Load(scenefilename) ) {
		return 2;
	}




    int contactpoints = 0;
    {
        // lock the environment to prevent data from changing
        EnvironmentMutex::scoped_lock lock(penv->GetMutex());

        vector<KinBodyPtr> vbodies;
        penv->GetBodies(vbodies);
        // get the first body
        if( vbodies.size() == 0 ) {
            RAVELOG_ERROR("no bodies loaded\n");
            return -3;
        }

        KinBodyPtr pbody = vbodies.at(0);
        vector<dReal> values;
        pbody->GetDOFValues(values);

        // set new values
        for(int i = 0; i < (int)vsetvalues.size() && i < (int)values.size(); ++i) {
            values[i] = vsetvalues[i];
        }

	values[RSY] = -1.0;
	values[REB] = 1.0;
        pbody->SetDOFValues(values,true);

        CollisionReportPtr report(new CollisionReport());
        penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
        if( pbody->CheckSelfCollision(report) ) {
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
        else {
            RAVELOG_INFO("body not in collision\n");
        }

        // get the transformations of all the links
        vector<Transform> vlinktransforms;
        pbody->GetLinkTransformations(vlinktransforms);
    }

	pause();

    RaveDestroy(); // destroy
    return contactpoints;
}
