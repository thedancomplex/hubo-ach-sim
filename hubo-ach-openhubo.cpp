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

using namespace OpenRAVE;
using namespace std;

void printhelp()
{
    RAVELOG_INFO("orcollision [--list] [--checker checker_name] [--joints #values [values]] body_model\n");
}

int main(int argc, char ** argv)
{

    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
    vector<dReal> vsetvalues;

    // parse the command line options
    int i = 1;
    

	// load the scene
    if( !penv->Load("openHubo/jaemiHubo.robot.xml") ) {
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

    RaveDestroy(); // destroy
    return contactpoints;
}
