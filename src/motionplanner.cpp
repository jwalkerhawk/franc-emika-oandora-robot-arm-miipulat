#include <motionplanner.h>

	
/// \brief Constructor
MotionPlanner::MotionPlanner(GazeboController* controller)
{
	
	//Hand over pointer to Gazebo controller
	mp_controller = controller;
	
	//Define joint limits w.r.t. Gazebo simulation
	Eigen::Matrix<double,7,1> q_min;
	q_min << 	-2.9,
				-1.76,
				-2.9,
				-3.07,
				-2.9,
				-0.02,
				-2.9;
				
	Eigen::Matrix<double,7,1> q_max;
	q_max << 	2.9,
				1.76,
				2.9,
				0,
				2.9,
				3.75,
				2.9;
	
	//Setup random number generators
    m_gen = std::mt19937(); 
    m_dis1 = std::uniform_real_distribution<>(q_min(0), q_max(0));
    m_dis2 = std::uniform_real_distribution<>(q_min(1), q_max(1));
    m_dis3 = std::uniform_real_distribution<>(q_min(2), q_max(2));
    m_dis4 = std::uniform_real_distribution<>(q_min(3), q_max(3));
    m_dis5 = std::uniform_real_distribution<>(q_min(4), q_max(4));
    m_dis6 = std::uniform_real_distribution<>(q_min(5), q_max(5));
    m_dis7 = std::uniform_real_distribution<>(q_min(6), q_max(6));
}

MotionPlanner::~MotionPlanner()
{	
	
}

std::vector<Eigen::MatrixXd> MotionPlanner::planMotion(Eigen::MatrixXd q_start,Eigen::MatrixXd q_goal)
{	
	
	int iter = 0;
	bool succes = false;
	double prop_size = 1 ;
	Eigen::MatrixXd rand_goal;
	std::vector<Eigen::MatrixXd> path;
	path.push_back(q_start);
	std::vector<int> nearest;
	nearest.push_back(-1);
	std::vector<Eigen::MatrixXd> fin_path;
	// picks the random sample path we are looking for
	while (iter!=10000){
		if ((iter%10 == 9|| iter%10 ==7 || iter%10 == 4)){
			rand_goal = q_goal;
		}
		else{
			rand_goal = drawRandomConfig();
		}
	// finds the nearset point to our random sample 
	Eigen::MatrixXd start = q_start;
	double compare = INFINITY;
	int parent_ind = 0;
	for (int i = 0; i<path.size(); i++){
		double distance = (rand_goal - path[i]).array().square().sum();
		if (distance < compare){
			start = path.at(i);
			compare = distance;
			parent_ind = i;
		}
	}
	
	// finds the sample goal to the specific length that is wanted
	double fin_distance = sqrt((rand_goal - path[parent_ind]).array().square().sum());
	if ( fin_distance > M_PI_2/prop_size ){
		rand_goal = (((rand_goal-start)*(M_PI_2/prop_size))/(rand_goal - start).norm()) + start;
		// std::cout <<  "found"<< std::endl;
	}
	
	// std::cout <<  "\\\\\\\\\\\\\\"<< std::endl;
	// std::cout <<  rand_goal<< std::endl;

	iter+=1;
	// if there is no colision then we add to the path and parent indices path
	bool check = true;
	for (int che=1; che < 25; che++){
		if (getCollisionState(((rand_goal-start)* double(1.0/che)) + start)){
			// std::cout << "collision"<< std::endl;
			check = false;
		}
	}
	if (check){
		nearest.push_back(parent_ind);
		path.push_back(rand_goal);
		// std::cout << path.size()<< std::endl;
		// if the proper path is found we build the path backwards
		if (rand_goal == q_goal){
			succes = true;
			fin_path.push_back(q_goal);

			while(parent_ind != -1){
				fin_path.push_back(path.at(parent_ind));
				parent_ind = nearest.at(parent_ind);
			}
			//reverse the list and break out of while loop
			std::reverse(fin_path.begin(),fin_path.end());
			break;

		}
	}
	}
	return fin_path;
	
}

Eigen::MatrixXd MotionPlanner::drawRandomConfig()
{	
	Eigen::MatrixXd q;
	q.resize(7,1);
	
	q << 	m_dis1(m_gen),
			m_dis2(m_gen),
			m_dis3(m_gen),
			m_dis4(m_gen),
			m_dis5(m_gen),
			m_dis6(m_gen),
			m_dis7(m_gen);
			
	return q;
	
}

bool MotionPlanner::getCollisionState(Eigen::MatrixXd q_check)
{	
	
	if(q_check.rows() != 7 || q_check.cols() != 1)
	{
		
		std::cout << "Wrong dimensions of robot configuration!" << std::endl;
		return false;
	}
	
	mp_controller->setRobotConfiguration(q_check);
	
	//Make sure the robot moved in simulation and the EE frame updated
	while(!mp_controller->isFrameValid())
	{
			
	}
			
	return mp_controller->getCollisionState();
	
}

