#include "subsystems/VisionReceiver.h"
#include "Robot.h"
#include <iostream>
#include <streambuf>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>


constexpr char VISION_PORT[] = "8081";

VisionReceiver::VisionReceiver() : Subsystem("VisionReceiver"),
visionDataStream(&visionDataStreamBuf) {
	setupSocket();
}

void VisionReceiver::setupSocket() {
	
	struct addrinfo hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = AI_PASSIVE;   /* For wildcard IP address */

	struct addrinfo *result;

	int error = getaddrinfo(nullptr, VISION_PORT, &hints, &result);
	if (error != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(error));
	}

	for (struct addrinfo *rp = result; rp != NULL; rp = rp->ai_next) {
               sockfd = socket(rp->ai_family, rp->ai_socktype,
                       rp->ai_protocol);
        if (sockfd != -1) {

			if (bind(sockfd, rp->ai_addr, rp->ai_addrlen) == 0) break;
			else {
				perror("failed to bind to vision socket");
				close(sockfd);
				sockfd = -1;
			}
		}
	}
	freeaddrinfo(result);

	if (sockfd == -1) {
		std::cout << "could not connect to vision socket" << std::endl;
	}
}

void VisionReceiver::Periodic() {

	while (true) {
	char buf[66537];
		ssize_t recieveSize = recvfrom(sockfd, buf, sizeof(buf) - 1, 
		MSG_DONTWAIT, nullptr, nullptr);
		if (recieveSize > 0) {
			buf[recieveSize] = '\0';
			visionDataStream << buf;
		}
		else break;
	}
	std::string line;
	while (std::getline(visionDataStream, line) && line[0] == '#') {
			int isPort, num;
			TargetData data;
			sscanf(line.c_str(), "#%d: isPort=%d distance=%lf tapeAngle=%lf robotAngle=%lf",
			&num, &isPort, &data.distance, &data.tapeAngle, &data.robotAngle);
			readTapes.push_back(data);
			
	}
	if(line[0]!='@'){
		perror("Vision data parse error, invalid header");
		goto CLEAR;
	}
	if (readTapes.size() == 0) {
		sscanf(line.c_str(), "@%d", &latency);
		//TODO: correct for latency
		AutoDrive::RobotPosition robPos = Robot::autoDrive.currentPosition;
		targetLocs.clear();
		TargetData i=readTapes.at(readTapes.size()-1);
		if(latency > 500){ //If data is really old
			goto CLEAR;
		}
		TargetLoc target;
		double wholeAngle = robPos.angle/180*M_PI + i.robotAngle;

		target.loc.x = robPos.loc.x + i.distance*sin(wholeAngle);
		target.loc.y = robPos.loc.y + i.distance*cos(wholeAngle);

		target.angle = i.tapeAngle;

		targetLocs.push_back(target);
		
		newData = true;
	}
	else {
	std::cout << "No data found" << std::endl;
	}
	CLEAR:
	readTapes.clear();
	//More stuff here?
}