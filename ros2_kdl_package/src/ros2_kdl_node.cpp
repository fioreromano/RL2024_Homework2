// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort")) //cmd_interface torque da aggiungere
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }
            
            // declare k parameter (trajectory type)
            declare_parameter("k", 1);  // Default value = 1 (circular with cubic polynomial)
            get_parameter("k", k_);
            RCLCPP_INFO(get_logger(), "Current trajectory type: %d", k_);

            if (k_ == 1) {
             RCLCPP_INFO(get_logger(), "Trajectory type: circular with cubic polynomial");
            } else if (k_ == 2) {
             RCLCPP_INFO(get_logger(), "Trajectory type: circular with trapezoidal velocity profile");
            } else if (k_ == 3) {
             RCLCPP_INFO(get_logger(), "Trajectory type: linear with cubic polynomial");
            } else if (k_ == 4) {
             RCLCPP_INFO(get_logger(), "Trajectory type: linear with trapezoidal velocity profile");
            } else {
             RCLCPP_INFO(get_logger(), "Invalid value for k. Defaulting to 1: circular with cubic polynomial.");
             k_ = 1;
            }
            
            // declare effort parameter (joint, cartesian)
            declare_parameter("effort", "joint"); 	// defaults to "joint"
            get_parameter("effort", effort_);
            RCLCPP_INFO(get_logger(),"Current effort control is: '%s' space control", effort_.c_str());

            if (!(effort_ == "joint" || effort_ == "cartesian"))
            {
                RCLCPP_INFO(get_logger(),"Selected effort control is not valid!"); return;
            }

            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            robot_temp=std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);  
            robot_temp->setJntLimits(q_min,q_max);       
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_temp.resize(nj); 
            joint_velocities_temp.resize(nj);
            joint_acc_.resize(nj);
            torque.resize(nj);
            torque_new.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){ //qui richiamo una funzione chiamata in 250
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            robot_temp->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            robot_temp->addEE(f_T_ee);
            robot_temp->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));


            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            std::cout << "The initial EE pose is: " << std::endl;  
            std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            
            // Initialize controller
            KDLController controller_(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));           
            end_position << init_position[0], -init_position[1], init_position[2];
 
            // Plan trajectory
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, radius = 0.05;
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, radius);
            trajectory_point p; 

            // Retrieve the first trajectory point
            if (k_==1) p = planner_.compute_trajectory_cir_pol(t);
            if (k_==2) p = planner_.compute_trajectory_cir_trap(t);
            if (k_==3) p = planner_.compute_trajectory_lin_pol(t);
            if (k_==4) p = planner_.compute_trajectory_lin_trap(t);

            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));

            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else
                if (cmd_interface_ == "velocity")
                    { 
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                        }
                    }

                else
                { 
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                    // Send joint torque commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = torque(i);
                        //desired_commands_[i] = torque_new(i);
                    }
                }


            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher()
        {

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 1.5; 
            int trajectory_len = 150; 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
            KDLController controller_(*robot_);
            

            if (t_ < total_time){
            
		trajectory_point p;
                // Retrieve the trajectory point
                if (k_==1) p = planner_.compute_trajectory_cir_pol(t_);
                if (k_==2) p = planner_.compute_trajectory_cir_trap(t_);
                if (k_==3) p = planner_.compute_trajectory_lin_pol(t_);
                if (k_==4) p = planner_.compute_trajectory_lin_trap(t_);

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; 
                desFrame.M = cartpos.M; 
                desFrame.p = toKDL(p.pos); 			//.p è la pos dell'origine della terna, .M è la rotazione

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; 
                    nextFrame.M = cartpos.M; 
                    nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(2*error))*dt;  

                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else
                    if (cmd_interface_ == "velocity")
                    { 
                        // Compute differential IK
                        Vector6d cartvel; 
                        cartvel << p.vel + 5*error, o_error; 
                        joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel; 
                        joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt; 
                    }
                    else {   

                        Eigen::Vector3d pos_eigen = p.pos;
     
                        KDL::Frame nextFrame;
                        nextFrame.M = cartpos.M;
                        KDL::Vector pos_kdl(pos_eigen(0), pos_eigen(1), pos_eigen(2));
                        nextFrame.p = pos_kdl;
             
                        KDL::Twist twist_vel_des = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
                        KDL::Twist twist_acc_des = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());

                        Eigen::Matrix<double,6,1> err_dot;     

                        o_error<< 0.0, 0.0, 0.0;

                        Vector6d cartvel, cartacc;
                        cartvel << p.vel + 4*error, o_error;
                        cartacc << p.acc, o_error;
                        Eigen::VectorXd J_dot_q_dot;
                        
                        // Compute differential IK
                        joint_velocities_temp.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel; 
                        joint_positions_temp.data = joint_positions_.data + joint_velocities_temp.data*dt; 
                        robot_temp->update(toStdVector(joint_positions_temp.data),toStdVector(joint_velocities_temp.data));
                        Vector6d err;
                        err << error,o_error;
                        J_dot_q_dot=robot_temp->getEEJacDotqDot();
                        Eigen::Vector3d p_dot_d(twist_vel_des.vel.data);
                        Eigen::Vector3d p_dot_e(robot_->getEEVelocity().vel.data);
                        Eigen::Matrix<double,3,1> e_dot_p = computeLinearError(p_dot_d,p_dot_e);
                 
                        Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(nextFrame.M.data );
                        Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
                         R_d = matrixOrthonormalization(R_d);
                         R_e = matrixOrthonormalization(R_e);
                        Eigen::Vector3d w_d(twist_vel_des.rot.data);
                        Eigen::Vector3d w_e(robot_->getEEVelocity().rot.data);
                        Eigen::Matrix<double,3,1> e_dot_o = computeOrientationVelocityError(w_d, w_e, R_d, R_e);
                        err_dot << e_dot_p, e_dot_o;

                        joint_acc_.data = pseudoinverse(robot_temp->getEEJacobian().data)*(cartacc - J_dot_q_dot + 40*err+1*err_dot);
                        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
 
                        //JOINT SPACE
                        if (effort_ == "joint") 
                        computed_torque = controller_.idCntr(joint_positions_temp, joint_velocities_temp, joint_acc_, 70, 2*sqrt(100));
                        //CARTESIAN SPACE
                        if (effort_ == "cartesian") 
                        computed_torque = controller_.idCntr(nextFrame,twist_vel_des,twist_acc_des,60,60,10,10);                        
                        torque = toKDLJntArray(computed_torque);

			/*
                        std::cout <<std::endl<<"torque                      ";
                        for (int k=0; k<7;k++)
                        std::cout << torque(k)<<"  ";


                        std::cout <<std::endl<<"joint_positions_desiderato  ";
                        for (int k=0; k<7;k++)
                        std::cout<< joint_positions_temp(k)<<"  ";

                        std::cout <<std::endl<<"joint_positions_            ";
                        for (int k=0; k<7;k++)
                        std::cout<< joint_positions_(k)<<"  ";

                        std::cout <<std::endl<<"joint_velocities_desiderato ";
                        for (int k=0; k<7;k++)
                        std::cout<< joint_velocities_temp(k)<<"  ";

                        std::cout <<std::endl<<"joint_velocities_           ";
                        for (int k=0; k<7;k++)
                        std::cout<< joint_velocities_(k)<<"  ";

                        std::cout <<std::endl<<"x_reale                     ";
                        for (int k=0; k<3;k++)
                        std::cout << cartpos.p(k)<<"  ";

                        std::cout <<std::endl<<"x_desiderato                ";
                        for (int k=0; k<3;k++)
                        std::cout <<  p.pos(k)<<"  "; */
                    
                        //std::cout <<std::endl;
                        //torque=robot_->getJsim() * (ddqd + 10*de + 20*e) + robot_->getCoriolis() + robot_->getGravity();                

                 }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else
                    if (cmd_interface_ == "velocity")
                    {
                        // Send joint velocity commands
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = joint_velocities_(i);
                        }
                    }
                    else
                    {
                        // Send joint torque commands
                        for (long int i = 0; i < torque.data.size(); ++i) {
                            desired_commands_[i] = torque(i);
                         }
                    }


                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }        

            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint velocity commands
                    if (cmd_interface_=="position")
                    {
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = joint_positions_(i); //the last value of the sensor (which corresponds to the end position that we want) 
											//will be used to keep the robot still
                        }
                    }
                    if (cmd_interface_=="velocity")
                    {
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = 0.0;
                        }
                    }
                    if (cmd_interface_=="effort")
                    {
                        Vector6d cartvelf, cartaccf;
                        cartvelf << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                        cartaccf << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                        Eigen::VectorXd J_dot_q_dot_;

                        joint_velocities_temp.data << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                        KDL::Frame pos_attuale = robot_->getEEFrame();
                        KDL::Frame lastFrame; 
                        lastFrame.p=pos_attuale.p; 
                        lastFrame.M = pos_attuale.M;

                        robot_->getInverseKinematics(lastFrame, joint_positions_temp);
                        robot_temp->update(toStdVector(joint_positions_temp.data),toStdVector(joint_velocities_temp.data));
                        Vector6d erro;
                        Eigen::Vector3d errore_finale;

                        errore_finale = computeLinearError(toEigen(lastFrame.p), Eigen::Vector3d(pos_attuale.p.data));
                        erro << errore_finale, 0.0, 0.0, 0.0;

                        J_dot_q_dot_=robot_->getEEJacDotqDot();
                        joint_acc_.data = pseudoinverse(robot_->getEEJacobian().data)*(cartaccf - J_dot_q_dot_ + 5*erro);

                        Eigen::VectorXd ddqd = joint_acc_.data;
                        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
 
                        computed_torque = controller_.idCntr(joint_positions_temp, joint_velocities_temp, joint_acc_, 70, 2*sqrt(100));
                        torque = toKDLJntArray(computed_torque);
                       
                        for (long int i = 0; i < torque.data.size(); ++i) {
                            desired_commands_[i] = torque(i);
                         }

                    }                
                    // Create msg and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_positions_temp;
        KDL::JntArray joint_velocities_temp;
        Eigen::VectorXd computed_torque;
        
        KDL::JntArray joint_acc_;
        KDL::JntArray torque;
        KDL::JntArray torque_new;
        //Eigen::VectorXd torque_new;
        std::shared_ptr<KDLRobot> robot_;
        std::shared_ptr<KDLRobot> robot_temp;
        std::shared_ptr<KDLController> controller_;
        KDLPlanner planner_;
        Eigen::Vector3d end_position; 

        int iteration_;
        bool joint_state_available_;
        double t_;
        unsigned int k_;
        std::string effort_;
        std::string cmd_interface_;
        KDL::Frame init_cart_pose_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
