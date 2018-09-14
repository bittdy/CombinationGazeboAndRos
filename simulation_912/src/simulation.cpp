#include <gazebo_msgs/ModelStates.h> 
#include <gazebo_msgs/ModelState.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <cstring>
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <cmath>

std::vector<std::string> modelName;
std::vector<geometry_msgs::Pose> poseArray;
std::vector<geometry_msgs::Twist> twistArray;

ros::Publisher modelStatesPub;
//ltl results--how to read these data autonomous?
//float[][] robot1Prefix = {{2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4}, {3, 4}, {4, 4}, {3, 4}, {2, 4}, {2, 3}, {2, 2}};
//float[][] robot1Suffix = {{2, 2}, {2, 3}, {2, 4}, {3, 4}, {4, 4}, {3, 4}, {2, 4}, {2, 3}, {2, 2}};

//float[][] robot2Prefix = {{2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 1}, {4, 0}, {3, 0}, {2, 0}};
//float[][] robot2Suffix = {{2, 0}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 1}, {4, 0}, {3, 0}, {2, 0}};

//float[][] robot3Prefix = {{2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4}, {1, 4}, {0, 4}, {1, 4}, {2, 4}, {2, 3}, {2, 2}, {2, 1}, {2, 0}};
//float[][] robot3Suffix = {{2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4}, {1, 4}, {0, 4}, {1, 4}, {2, 4}, {2, 3}, {2, 2}, {2, 1}, {2, 0}};

// std::vector<float[2]> request;
// std::vector<float[2]> reply;

// request.push_back({3,4});
// reply.push_back({4,2});
// reply.push_back({0,4});
/////////////////////////////////////////////////
struct model
{
	std::string name;
	geometry_msgs::Point pose;
	geometry_msgs::Quaternion orientation;
	geometry_msgs::Twist twist;
};

// struct goalPoints
// {
// 	std::vector<geometry_msgs::Point> goal;
// 	std::vector<geometry_msgs::Point> request;
// 	std::vector<geometry_msgs::Point> reply;
// };

// std::vector<goalPoints> robot1;
// std::vector<goalPoints> robot2;
// std::vector<goalPoints> robot3;
//process the origin data
std::vector<model> infoProcess()
{
	std::vector<model> modelVec;
	modelVec.clear();
	model modelBuff;
	for(int i = 0; i < modelName.size(); i++)
	{
		modelBuff.name = modelName[i];
		modelBuff.pose = poseArray[i].position;
		modelBuff.orientation = poseArray[i].orientation;
		modelBuff.twist = twistArray[i];
		modelVec.push_back(modelBuff);
	}
	return modelVec;
}


//convert the angle to quater, and only rotate respect to z-rxis
void anglToQuater(float* quater,float angle)
{
	quater[0] = 0*sin(angle/2);
	quater[1] = 0*sin(angle/2);
	quater[2] = 1*sin(angle/2);
	quater[3] = cos(angle/2);
}

void multiQuater(float* q3,float* q1,float* q2)
{
	//x = (w1*x2+x1*w2+y1*z2-z1*y2)
	*(q3) = ((*(q1 + 3))*(*(q2))+(*(q1))*(*(q2 + 3))+(*(q1 + 1))*(*(q2 + 2))-(*(q1 + 2))*(*(q2 + 1)));
	//y = (w1*y2-x1*z2+y1*w2+z1*x2)
	*(q3 + 1) = ((*(q1 + 3))*(*(q2 + 1))-(*(q1))*(*(q2 + 2))+(*(q1 + 1))*(*(q2 + 3))+(*(q1 + 2))*(*(q2)));
	//z = (w1*z2+x1*y2-y1*x2+z1*w2)
	*(q3 + 2) = ((*(q1 + 3))*(*(q2 + 2))+(*(q1))*(*(q2 + 1))-(*(q1 + 1))*(*(q2))+(*(q1 + 2))*(*(q2 + 3)));
	//w = (w1*w2-x1*x2-y1*y2-z1*z2)
	*(q3 + 3) = ((*(q1 + 3))*(*(q2 + 3))-(*(q1))*(*(q2))-(*(q1 + 1))*(*(q2 + 1))-(*(q1 + 2))*(*(q2 + 2)));
}


void chatterCallback(const gazebo_msgs::ModelStates& msg)
{
	modelName = msg.name;
	poseArray = msg.pose;
	twistArray = msg.twist;	
	std::vector<model> modelVec = infoProcess();
	gazebo_msgs::ModelState modelStateIt;
	for(std::vector<model>::iterator it = modelVec.begin(); it != modelVec.end(); it++)
	{
		if((*it).name == "pioneer2dx_0")
		{
			modelStateIt.model_name = "pioneer2dx_0";
			modelStateIt.twist = (*it).twist;
			modelStateIt.pose.position.x = (*it).pose.x;
			//1000hz,compute the velocity
			modelStateIt.pose.position.y = (*it).pose.y + 0.008;
			modelStateIt.pose.position.z = (*it).pose.z;
			float* quater = new float[4];
			anglToQuater(quater,0.01);
			float* currentQuater = new float[4];
			*(currentQuater) = (*it).orientation.x;
			*(currentQuater + 1) = (*it).orientation.y;
			*(currentQuater + 2) = (*it).orientation.z;
			*(currentQuater + 3) = (*it).orientation.w;
			float* resQua = new float[4];
			multiQuater(resQua,currentQuater,quater);
			modelStateIt.pose.orientation.x = *(resQua);
			modelStateIt.pose.orientation.y = *(resQua + 1);
			modelStateIt.pose.orientation.z = *(resQua + 2);
			modelStateIt.pose.orientation.w = *(resQua + 3);
		}
	}
	modelStatesPub.publish(modelStateIt);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "simulation");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1000, chatterCallback);
	modelStatesPub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1000);
	ros::spin();
	return 0;
}
