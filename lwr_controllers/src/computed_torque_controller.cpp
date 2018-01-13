// #include <angles/angles.h>
// #include <pluginlib/class_list_macros.h>
// #include <algorithm>
// #include <kdl/tree.hpp>
// #include <kdl/chainfksolvervel_recursive.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <urdf/model.h>
// 
// #include <lwr_controllers/computed_torque_controller.h>
// 
// namespace lwr_controllers {
// 
// ComputedTorqueController::ComputedTorqueController() {}
// 
// ComputedTorqueController::~ComputedTorqueController() {}
// 
// bool ComputedTorqueController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
// {
//     KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
//     K_.resize(kdl_chain_.getNrOfJoints());
//     D_.resize(kdl_chain_.getNrOfJoints());   
//     q_des_.resize(kdl_chain_.getNrOfJoints());
//     tau_des_.resize(kdl_chain_.getNrOfJoints());
//  
//     for (size_t i = 0; i < joint_handles_.size(); i++)
//     {
//         tau_des_(i) = joint_handles_[i].getPosition();
//         K_(i) = joint_stiffness_handles_[i].getPosition();
//         D_(i) = joint_damping_handles_[i].getPosition();
//         q_des_(i) = joint_set_point_handles_[i].getPosition();
//     }
// 
//     ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );
// 
//     for (int i = 0; i < joint_handles_.size(); ++i) {
//         if ( !nh_.getParam("stiffness_gains", K_(i) ) ) {
//             ROS_WARN("Stiffness gain not set in yaml file, Using %f", K_(i));
//         }
//     }
//     for (int i = 0; i < joint_handles_.size(); ++i) {
//         if ( !nh_.getParam("damping_gains", D_(i)) ) {
//             ROS_WARN("Damping gain not set in yaml file, Using %f", D_(i));
//         }
//     }
// 
//     typedef  const std_msgs::Float64MultiArray::ConstPtr& msg_type;
//     sub_stiffness_ = nh_.subscribe<ComputedTorqueController, msg_type>("stiffness", 1, boost::bind(&ComputedTorqueController::setParam, this, _1, &K_, "K"));
//     sub_damping_ = nh_.subscribe<ComputedTorqueController, msg_type>("damping", 1, boost::bind(&ComputedTorqueController::setParam, this, _1, &D_, "D"));
//     sub_add_torque_ = nh_.subscribe<ComputedTorqueController, msg_type>("additional_torque", 1, boost::bind(&ComputedTorqueController::setParam, this, _1, &tau_des_, "AddTorque"));
//     sub_posture_ = nh_.subscribe("command", 1, &ComputedTorqueController::command, this);
// 
//     return true;
// 
// 
// }
// 
// void ComputedTorqueController::starting(const ros::Time& time)
// {
//     // Initializing stiffness, damping, ext_torque and set point values
//     for (size_t i = 0; i < joint_handles_.size(); i++) {
//         tau_des_(i) = 0.0;
//         q_des_(i) = joint_handles_[i].getPosition();
//     }
// 
// 
// }
// 
// void ComputedTorqueController::update(const ros::Time& time, const ros::Duration& period)
// {
// 
//     //Compute control law. This controller sets all variables for the JointImpedance Interface from kuka
//     for (size_t i = 0; i < joint_handles_.size(); i++)
//     {
//         joint_handles_[i].setCommand(tau_des_(i));
//         joint_stiffness_handles_[i].setCommand(K_(i));
//         joint_damping_handles_[i].setCommand(D_(i));
//         joint_set_point_handles_[i].setCommand(q_des_(i));
// 	std::cout << q_des_(i);
//     }
//     std::cout <<std::endl;
// 
// }
// 
// 
// void ComputedTorqueController::command(const std_msgs::Float64MultiArray::ConstPtr &msg) {
//     if (msg->data.size() == 0) {
//         ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
//     }
//     else if ((int)msg->data.size() != joint_handles_.size()) {
//         ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
//         return;
//     }
//     else
//     {
//               ROS_INFO("Received new position");
//         for (unsigned int j = 0; j < joint_handles_.size(); ++j)
//             q_des_(j) = msg->data[j];
//     }
// 
// }
// 
// void ComputedTorqueController::setParam(const std_msgs::Float64MultiArray_< std::allocator< void > >::ConstPtr& msg, KDL::JntArray* array, std::string s)
// {
//     if (msg->data.size() == joint_handles_.size())
//     {
//         for (unsigned int i = 0; i < joint_handles_.size(); ++i)
//         {
//             (*array)(i) = msg->data[i];
//         }
//     }
//     else
//     {
//         ROS_INFO("Num of Joint handles = %lu", joint_handles_.size());
//     }
// 
//     ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());
// 
//     ROS_INFO("New param %s: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", s.c_str(),
//              (*array)(0), (*array)(1), (*array)(2), (*array)(3), (*array)(4), (*array)(5), (*array)(6));
// }
// 
// } // namespace
// 
// PLUGINLIB_EXPORT_CLASS( lwr_controllers::ComputedTorqueController, controller_interface::ControllerBase)

/* @@@@@@@@@@@@@@@@@ COMPUTED TORQUE @@@@@@@@@@@@@@@@@@@@@@@@@*/

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <lwr_controllers/computed_torque_controller.h>

namespace lwr_controllers 
{
	ComputedTorqueController::ComputedTorqueController() {}
	ComputedTorqueController::~ComputedTorqueController() {}

	bool ComputedTorqueController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
        
		id_solver_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

		cmd_states_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_old_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kv_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());
        joint_initial_states_.resize(kdl_chain_.getNrOfJoints());
        current_cmd_.resize(kdl_chain_.getNrOfJoints());
	
// 	for(size_t i=0; i<joint_handles_.size(); i++) 
//   		{
// 
//   			Kp_(i) = 300.0;
//   			Kv_(i) = 0.7;
//     	}

		sub_posture_ = nh_.subscribe("command", 1, &ComputedTorqueController::command, this);
		sub_gains_ = nh_.subscribe("set_gains", 1, &ComputedTorqueController::set_gains, this);

		
// 		// starting stuff is done here
	  ROS_INFO("Sleeping...");
// 	  ros::Duration(5).sleep();
	  ROS_INFO("Waking up...");
  		// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
		  
// 		  Kp_(i) = 300.0;
// 		  Kv_(i) = 0.7;
			Kp_(i) = 300.0;
  			Kv_(i) = 0.7;
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
		joint_des_states_.qdot(i) = 0.0;
		joint_des_states_.qdotdot(i) = 0.0;
		joint_initial_states_(i) = joint_des_states_.q(i);
		cmd_states_(i) = joint_des_states_.q(i);
    	}

    	lambda = 0.1;	// lower values: flatter
    	cmd_flag_ = 0;	
    	step_ = 0;

	tau_read.resize(7);
	tau_read = {-0.9200000166893005, -42.77000045776367, 0.05000000074505806, 11.390000343322754, -0.25999999046325684, -0.4099999964237213, -0.07000000029802322};
	
    	ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );
	
	// logging
	log.open("/home/manuelb/log.txt",std::ios::trunc);
	log_q.open("/home/manuelb/log_q.txt",std::ios::trunc);

	start_time = ros::Time::now().toSec();

// 		// starting stuff end
		return true;		
	}

	void ComputedTorqueController::starting(const ros::Time& time)
	{
// 	  ROS_INFO("Sleeping...");
// 	  ros::Duration(5).sleep();
// 	  ROS_INFO("Waking up...");
//   		// get joint positions
//   		for(size_t i=0; i<joint_handles_.size(); i++) 
//   		{
// 		  
// // 		  Kp_(i) = 300.0;
// // 		  Kv_(i) = 0.7;
// 			Kp_(i) = 300.0;
//   			Kv_(i) = 0.7;
//     		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
//     		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
//     		joint_msr_states_.qdotdot(i) = 0.0;
//     		joint_des_states_.q(i) = joint_msr_states_.q(i);
// 		joint_des_states_.qdot(i) = 0.0;
// 		joint_des_states_.qdotdot(i) = 0.0;
// 		joint_initial_states_(i) = joint_des_states_.q(i);
// 		cmd_states_(i) = joint_des_states_.q(i);
//     	}
// 
//     	lambda = 0.1;	// lower values: flatter
//     	cmd_flag_ = 0;	
//     	step_ = 0;
// 
//     	ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );
// 	
// 	// logging
// 	log.open("/home/manuelb/log.txt",std::ios::trunc);
// 	start_time = ros::Time::now().toSec();

    }

    void ComputedTorqueController::update(const ros::Time& time, const ros::Duration& period)
    {
//       		std::cout << period << std::endl;
      if(step_==0) {
	  ROS_INFO("First time in update. Sleeping...");
// 	  ros::Duration(5).sleep();
	  ROS_INFO("Waking up...");
	  step_++;
      }
//       std::cout << "Step_: " << step_ << std::endl;
    	// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
// 		std::cout << joint_msr_states_.q(i) << " ";
		joint_des_states_.q(i) = joint_msr_states_.q(i);
		joint_des_states_.qdot(i) = 0.0;
		joint_des_states_.qdotdot(i) = 0.0;
// 		joint_initial_states_(i) = joint_des_states_.q(i);
		cmd_states_(i) = joint_des_states_.q(i);
    	}
    	
//     	std::cout <<  std::endl;

    	if(cmd_flag_)
    	{
            if(step_ == 0)
            {
                joint_initial_states_ = joint_msr_states_.q;
            }
            // reaching desired joint position using a hyperbolic tangent function
            double th = tanh(M_PI-lambda*step_);
            double ch = cosh(M_PI-lambda*step_);
            double sh2 = 1.0/(ch*ch);
            
            for(size_t i=0; i<joint_handles_.size(); i++)
            {
                // TODO: take into account also initial/final velocity and acceleration
                current_cmd_(i) = cmd_states_(i) - joint_initial_states_(i);
                joint_des_states_.q(i) = current_cmd_(i)*0.5*(1.0-th) + joint_initial_states_(i);
                joint_des_states_.qdot(i) = current_cmd_(i)*0.5*lambda*sh2;
                joint_des_states_.qdotdot(i) = current_cmd_(i)*lambda*lambda*sh2*th;
// 		joint_des_states_.q(i) = joint_initial_states_(i);
//                 joint_des_states_.qdot(i) = 0.0;
//                 joint_des_states_.qdotdot(i) = 0.0;
            }
            ++step_;

    		if(joint_des_states_.q == cmd_states_)
    		{
    			cmd_flag_ = 0;	//reset command flag
    			step_ = 0;
    			ROS_INFO("Posture OK");
    		}
    	}

    	// computing Inertia, Coriolis and Gravity matrices
    	id_solver_->JntToMass(joint_msr_states_.q, M_);
    	id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
    	id_solver_->JntToGravity(joint_msr_states_.q, G_);
// 	id_solver_->JntToGravity(joint_des_states_.q, G_);

        // PID controller
        KDL::JntArray pid_cmd_(joint_handles_.size());
        // compensation of Coriolis and Gravity
        KDL::JntArray cg_cmd_(joint_handles_.size());
        for(size_t i=0; i<joint_handles_.size(); i++)
        {
            // control law
//              pid_cmd_(i) = joint_des_states_.qdotdot(i) + Kv_(i)*(joint_des_states_.qdot(i) - joint_msr_states_.qdot(i)) + Kp_(i)*(joint_des_states_.q(i) - joint_msr_states_.q(i));
//             cg_cmd_(i) = C_(i)*joint_msr_states_.qdot(i) + G_(i);
	  pid_cmd_(i) = Kv_(i)*(joint_des_states_.qdot(i) - joint_msr_states_.qdot(i)) + Kp_(i)*(joint_des_states_.q(i) - joint_msr_states_.q(i));
// 	  std::cout << "pid_cmd(" << i << "): " << pid_cmd_(i) << std::endl;
            cg_cmd_(i) = G_(i);
        }
//         tau_cmd_.data = M_.data * pid_cmd_.data;
// 	if(step_ < 2)
// 	{
// 	  tau_cmd_.data = M_.data * pid_cmd_.data;	  
 	  tau_cmd_.data = pid_cmd_.data;	  
        KDL::Add(tau_cmd_,cg_cmd_,tau_cmd_);

// 	}
// 	else
// 	{
// 	  double alpha = 0.99;
// 	  tau_cmd_.data = tau_cmd_old_.data*alpha + (1-alpha)* M_.data * pid_cmd_.data;
// 	}
// 	tau_cmd_old_.data = tau_cmd_.data;
//         KDL::Add(tau_cmd_,cg_cmd_,tau_cmd_);
        
//         if(step_>=0)
// 	{
	double now = ros::Time::now().toSec() - start_time;
	log << now << ", ";
	log_q << now << ", ";
        for(size_t i=0; i<joint_handles_.size(); i++)
        {
	  tau_cmd_(i) = tau_read.at(i);
            joint_handles_[i].setCommand(tau_cmd_(i));
	    log << joint_handles_[i].getCommand() << ", ";
	    log_q << joint_handles_[i].getPosition() << ", ";
        }
        log << std::endl;
	log_q << std::endl;
// 	}
        
    }

    void ComputedTorqueController::command(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
      ROS_ERROR("Got Message!");
    	if(msg->data.size() == 0)
    		ROS_INFO("Desired configuration must be of dimension %lu", joint_handles_.size());
    	else if(msg->data.size() != joint_handles_.size())
    	{
    		ROS_ERROR("Posture message had the wrong size: %u", (unsigned int)msg->data.size());
    		return;
    	}
    	else
    	{
    		for (unsigned int i = 0; i<joint_handles_.size(); i++)
    			cmd_states_(i) = msg->data[i];

    		cmd_flag_ = 1;
            // when a new command is set, steps should be reset to avoid jumps in the update
            step_ = 0;
    	}

	}

	void ComputedTorqueController::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{
		if(msg->data.size() == 2*joint_handles_.size())
		{
			for(unsigned int i = 0; i < joint_handles_.size(); i++)
			{
				Kp_(i) = msg->data[i];
				Kv_(i) = msg->data[i + joint_handles_.size()];
			}
		}
		else
			ROS_INFO("Number of Joint handles = %lu", joint_handles_.size());

		ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

		ROS_INFO("New gains Kp: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kp_(0), Kp_(1), Kp_(2), Kp_(3), Kp_(4), Kp_(5), Kp_(6));
		ROS_INFO("New gains Kv: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kv_(0), Kv_(1), Kv_(2), Kv_(3), Kv_(4), Kv_(5), Kv_(6));

	}
}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::ComputedTorqueController, controller_interface::ControllerBase)
