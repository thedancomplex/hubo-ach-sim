#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace OpenRAVE;
using namespace std;

int main( int argc, char ** argv) {

	int hflag = 0;
	int c;
	string scenefilename = "tmp";

	while ((c = getopt (argc, argv, "bs")) != -1) {
		switch (c) {
			case 'h':
				hflag = 1;
				scenefilename = sprintf( scenefilename, "%s", argv[1] );
				break;
			default:
				abort();
		}
	}

	printf(scenefilename);

}


