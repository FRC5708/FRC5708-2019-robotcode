#include "subsystems/VisionReceiver.h"
#include "Robot.h"
#include <iostream>
#include <streambuf>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>


constexpr char VISION_PORT[] = "5808";

VisionReceiver::VisionReceiver() : Subsystem("VisionReceiver") {
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
	++dataAge;
	//std::cout << "VisionReciever::Periodic : " << std::endl;
	
	std::vector<TargetData> readTapes;
	
	std::stringbuf visionDataStreamBuf;
	std::iostream visionDataStream(&visionDataStreamBuf);

	char buf[66537];
	ssize_t recieveSize = recvfrom(sockfd, buf, sizeof(buf) - 1, 
	MSG_DONTWAIT, nullptr, nullptr);
	if (recieveSize > 0) {
		buf[recieveSize] = '\0';
		visionDataStream << buf;

		std::string line;
		while (std::getline(visionDataStream, line)) {
			if (line[0] == '#') {
				std::cout << line << std::endl;
				int isPort, num;
				TargetData data;
				sscanf(line.c_str(), "#%d: isPort=%d distance=%lf tapeAngle=%lf robotAngle=%lf",
				&num, &isPort, &data.distance, &data.tapeAngle.value, &data.robotAngle.value);
				readTapes.push_back(data);
			}
			else if(line[0] == '@') {
				sscanf(line.c_str(), "@%d", &processingTime);
			}
			else {
				perror("Vision data parse error, invalid header");
				goto CLEAR;
			}
		}
	}
	else if (recieveSize < 0 && errno != EWOULDBLOCK) {
		perror("vision data recieve error");
		return;
	}
	// else do nothing
	
	if (readTapes.size() != 0) {
		targetLocs.clear();
		dataAge = 0;
		
		// Milliseconds. Network, camera, etc.
		constexpr double extraLatency = 20;

		if(processingTime + extraLatency > 1000){ //If data is really old
			goto CLEAR;
		}
        AutoDrive::RobotPosition robPos = Robot::autoDrive.getPastPos(processingTime + extraLatency);
		// What if there is some garbage data very close to the robot?
		// That's why we don't just pick the closest target.
		for (auto i : readTapes) {
			TargetLoc target;
			Radian wholeAngle = i.robotAngle + robPos.angle;

			target.loc.x = robPos.loc.x + i.distance*sin(wholeAngle)
			 + (ROBOT_LENGTH / 2 - 6)*sin(Radian(robPos.angle)) -
			  (ROBOT_WIDTH / 2 - 1)*cos(Radian(robPos.angle));
			target.loc.y = robPos.loc.y + i.distance*cos(wholeAngle)
			 + (ROBOT_LENGTH / 2 - 6)*cos(Radian(robPos.angle))
			 - (ROBOT_WIDTH / 2 - 1)*sin(Radian(robPos.angle));

			target.angle = i.tapeAngle + i.robotAngle + robPos.angle;

			targetLocs.push_back(target);
			std::cout << "got target at: <" << target.loc.x << ", " << target.loc.y << ">" << std::endl;
		} 
		newData = true;
	}
	else {
	//std::cout << "No data found" << std::endl;
	}
	CLEAR:
	readTapes.clear();
	//std::cout << "@CLEAR:" << std::endl;
	//More stuff here?
}