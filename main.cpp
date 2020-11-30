#include <gazebocontroller.h>
#include <motionplanner.h>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <random>
#include <unistd.h>

// Main code of the assignment framework
int main(int _argc, char **_argv)
{
	
	//--Initialization--
	
	// Create Gazebo controller to interact with simulation environment
	GazeboController controller;
	MotionPlanner planner(&controller);
	
	//Define configurations
	Eigen::Matrix<double,7,1> q_init;
	Eigen::Matrix<double,7,1> q_shelf;
	Eigen::Matrix<double,7,1> q_top;
	
	q_init = controller.getCurrentRobotConfiguration();
	
	q_shelf << 0.75,
		      0.9,
		      0,
		      -1.1,
		      0,
		      2,
		      0;
		      
	q_top << 0.4,
			 0.2,
			 -0.3,
			 -0.85,
			 0,
			 1.1,
			 0;
	
	Eigen::Matrix<double,7,1> q_start;		
	Eigen::Matrix<double,7,1> q_goal;
		      
	std::cout << "Choose one out of three paths (1,2,3): ";
	int path_id;
	std::cin >> path_id;
	std::cout << std::endl;
	
	switch(path_id) {
		case 1:
			q_start = q_init;
			q_goal = q_shelf;
			break;
		case 2:
			q_start = q_shelf;
			q_goal = q_top;
			break;
		case 3:
			q_start = q_top;
			q_goal = q_init;
			break;
		default:
			q_start = q_init;
			q_goal = q_shelf;
	}
	
	
	//--Motion planning--
	
	std::vector<Eigen::MatrixXd> path;
	
	path = planner.planMotion(q_start,q_goal);	
	
	std::cout << "Press any key to apply the planned path" << std::endl;
	std::cin.ignore();
	std::cin.get();
	
	//--Run through planned path--
	bool collision = false;
	int interpolation_steps = 100;
	Eigen::MatrixXd q_cur;
	Eigen::MatrixXd q_next;
	Eigen::MatrixXd q_inter;
	
	if(path.size() != 0)
	{
		for(int j = 0; j < path.size() - 1; j++)
		{
			q_cur = path.at(j);
			q_next = path.at(j+1);
			
			
			for(int i = 0; i < interpolation_steps + 1; i++)
			{
				q_inter = q_cur + (i)*(q_next - q_cur)/interpolation_steps;
				controller.setRobotConfiguration(q_inter);
			
				//Make sure the robot moved in simulation and the EE frame updated
				while(!controller.isFrameValid())
				{
					
				}
				
				if(collision == false)
					collision = controller.getCollisionState();
					
				usleep(100);
				
			}
		}			
	}
	
		
	if(path.size() == 0)
		std::cout << "Path is empty!" << std::endl;
	else if(path.back() != q_goal)
		std::cout << "Path does not reach goal configuration!" << std::endl;
	else
	{
		if(collision)
			std::cout << "Path is not collision free!" << std::endl;		
		else
			std::cout << "Path is collision free!" << std::endl;
	}

	std::cout << "Press any key to reset simulation and exit program" << std::endl;
	std::getchar();
	
	// Move robot to initial configuration (all angles are zero)
	controller.setRobotConfiguration(q_start);
	
	//Make sure the robot moved in simulation and the EE frame updated
	while(!controller.isFrameValid())
	{
	}
    
    
    return 0;
}
