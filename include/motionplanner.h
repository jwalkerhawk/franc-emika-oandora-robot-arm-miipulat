#pragma once

#include <Eigen/Dense>
#include <gazebocontroller.h>

/// \brief This class provides functionalities to plan motions for the Panda Manipulator
/// This class is mostly empty and needs to be implemented throughout the different assignment questions
class MotionPlanner
{
    
    public:
		
		//Constructor and Destructor
		MotionPlanner(GazeboController* controller);
		~MotionPlanner();
		
		
		// Function to plan a collision free path that moves the simulated robot from q_start to q_goal (both are 7x1 column vectors, specifying robot configurations)
		// Output: std::vector containing a sequence of configurations from that the path is constructed (first entry should be q_start and last entry should be q_goal)
		std::vector<Eigen::MatrixXd> planMotion(Eigen::MatrixXd q_start,Eigen::MatrixXd q_goal);
		
		
     
	private:
	
	
	// Function to draw a random configuration for the simulated robot, respecting its joint limits
	// Output: 7x1 column vector that is specifying a random robot configuration within its configuration space
    Eigen::MatrixXd drawRandomConfig();
    
     
    // Function to obtain the collision state of a given configuration q_check (7x1 column vector)
	// Output: Boolean value indicating the collision state of the robot in configuration q_check (true - in collision; false - no collision)
    bool getCollisionState(Eigen::MatrixXd q_check);
	
	//Pointer to Gazebo Controller
	GazeboController* mp_controller;
	
	//Random number generators
    std::mt19937 m_gen;
    std::uniform_real_distribution<> m_dis1;
    std::uniform_real_distribution<> m_dis2;
    std::uniform_real_distribution<> m_dis3;
    std::uniform_real_distribution<> m_dis4;
    std::uniform_real_distribution<> m_dis5;
    std::uniform_real_distribution<> m_dis6;
    std::uniform_real_distribution<> m_dis7;
      
    
    
	
		
};

