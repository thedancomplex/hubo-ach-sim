default: hubo-ach-openhubo

hubo-main: hubo-ach-openhubo
	gcc -o $@ $< -lach -lrt -l /usr/include/openrave-0.7

clean: 
	rm hubo-ach-openhubo
