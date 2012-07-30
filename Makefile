default: hubo-ach-openhubo

hubo-ach-openhubo: hubo-ach-openhubo.cpp
	g++ -I /usr/include/openrave-0.6/ -I ../hubo-ACH -o $@ $< -lopenrave -lach -lrt
clean: 
	rm hubo-ach-openhubo
