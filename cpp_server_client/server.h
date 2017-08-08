/*

	Header file for Server class
	Allows the creation of server objects and member functions

*/
#ifndef SERVER_H
#define SERVER_H

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <fstream>
#include <strings.h>
#include <stdlib.h>
#include <string>
#include <pthread.h>

// pcl libraries
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace pcl;
/*
	Class name: Server
	Function: creates a server using a default port. After a connection is established, then the server will execute a task
*/
class Server {

	// public Member variables
	public:
		
	    int pId, portNo, listenFd;
		socklen_t len; //store size of the address
		//bool loop = false;
		struct sockaddr_in svrAdd, clntAdd;
		
		pthread_t threadA[3];	// establishes multithreading but removed methods related to this
	
	public:
	
		// constructor
		Server();
		
		// public member functions
		bool startServer(void);	
		void serverTask(PointXYZRGB point);

	private:
		// private member functions
		int bindSocket(void);
		int setSocketAttributes(void);


};

#endif
