default: hubo-ach-openhubo

hubo-ach-openhubo: hubo-ach-openhubo.cpp
	g++ -I /usr/include/openrave-0.6/ -I ../hubo-ACH -o $@ $< -lach -lrt -lboost_thread -lopenrave0.6 -lopenrave-core0.6
clean: 
	rm hubo-ach-openhubo
