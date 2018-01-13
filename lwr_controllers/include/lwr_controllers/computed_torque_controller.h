// /*
// #ifndef LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H
// #define LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H
// 
// #include "KinematicChainControllerBase.h"
// 
// #include <visualization_msgs/Marker.h>
// #include <std_msgs/Float64MultiArray.h>
// 
// #include <boost/scoped_ptr.hpp>
// 
// /*
// 	tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)
// 
// */
// 
// namespace lwr_controllers
// {
// 
// 	class ComputedTorqueController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
// 	{
// 	public:
// 
// 		ComputedTorqueController();
// 		~ComputedTorqueController();
// 
// 		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
// 
// 		void starting(const ros::Time& time);
// 
// 		void update(const ros::Time& time, const ros::Duration& period);
// 		void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
// 		void setParam(const std_msgs::Float64MultiArray::ConstPtr &msg, KDL::JntArray* array, std::string s);
//         
// 	private:
// 
// 		ros::Subscriber sub_stiffness_, sub_damping_, sub_add_torque_;
// 		ros::Subscriber sub_posture_;
// 
// 		KDL::JntArray q_des_;
// 		KDL::JntArray tau_des_;
// 		KDL::JntArray K_, D_;
// 
// 	};
// 
// } // namespace
// 
// #endif*/


#ifndef LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H
#define LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>

// for simple logging
#include <iostream>
#include <fstream>

namespace lwr_controllers
{
	class ComputedTorqueController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:

		ComputedTorqueController();
		~ComputedTorqueController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);

	private:

		ros::Subscriber sub_posture_;
		ros::Subscriber sub_gains_;
        
		KDL::JntArray cmd_states_;
		int cmd_flag_;	// discriminate if a user command arrived
		double lambda;	// flattening coefficient of tanh
		int step_;		// step used in tanh for reaching gradually the desired posture
		KDL::JntArray joint_initial_states_; // joint as measured at the beginning of the control action
		KDL::JntArray current_cmd_; // command value as delta to be added to joint_initial_states_

		KDL::JntArray tau_cmd_;
		KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
		KDL::JntArray Kp_, Kv_;	//Position and Velocity gains

		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

		std::vector<double> tau_read;
		
		double time_ctrl;
		KDL::JntArray tau_cmd_old_;
		double start_time;
		// logging
		std::ofstream log;
		std::ofstream log_q;
	};
}

#endif