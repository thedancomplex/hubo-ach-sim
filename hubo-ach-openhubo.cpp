#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>

using namespace OpenRAVE;
using namespace std;

int main( int argc, char ** argv) {
	if( argc < 2) {
		printhelp();
		return -1;	/// no robots to load
	}

	RaveInitialize(true); // start openrave core
	EnvironmentBasePtr penv = RaveCreateEnviroment();	// create the main enviroment
}
