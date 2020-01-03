#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

namespace gazebo
{
	class husky : public ModelPlugin
	{
		private: physics::ModelPtr model;
		private: physics::JointController * jcX;
		private: physics::JointController * jcY;
		private: physics::JointController * jcYaw;
		private: physics::JointPtr jX;
		private: physics::JointPtr jY;
		private: physics::JointPtr jYaw;

		private: event::ConnectionPtr updateConnection;
		private: ros::NodeHandle nh;
		private: ros::Subscriber sub;

		private: float data_buf[3];
		private: int time;

		public: husky(){}
		public: ~husky(){
			ROS_INFO("Done");
			this->nh.shutdown();
		}

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)
		{

			ROS_INFO("Load husky!");

			int argc = 0;
			char** argv = NULL;

			ros::init(argc, argv, "husky");
			// sub = nh.subscribe("/joint_states", 1, &husky::cb, this);
			// sub = nh.subscribe("/tf", 1, &husky::cb, this);
			sub = nh.subscribe("dynamop/path_pose", 1, &husky::cb, this);

			model = _parent;
			jcX = new physics::JointController(model);
			jcY = new physics::JointController(model);
			jcYaw = new physics::JointController(model);

			jX = model->GetJoint("base_x_joint");
			jY = model->GetJoint("base_y_joint");
			jYaw = model->GetJoint("base_yaw_joint");


			updateConnection = event::Events::ConnectWorldUpdateBegin(
										boost::bind(&husky::OnUpdate,
										this, _1));
			time = 0;
			data_buf[0] = 0;
			data_buf[1] = 0;
			data_buf[2] = 0;
		}

		// public: void cb(const sensor_msgs::JointState::ConstPtr &msg)
		// public: void cb(const tf2_msgs::TFMessage::ConstPtr &msg)
		public: void cb(const std_msgs::Float32MultiArray::ConstPtr &msg)
		{
			// ROS_INFO("run cb()");
			// Subscribe /joint_states
			// data_buf[0] = msg->position[0];
			// data_buf[1] = msg->position[1];
			// data_buf[2] = msg->position[2];

			// Subscribe /tf
			// data_buf[0] = msg->transforms.at(0).transform.translation.x;
			// data_buf[1] = msg->transforms.at(1).transform.translation.y;
			// data_buf[2] = msg->transforms.at(2).transform.rotation.w;
			// if(data_buf[2] > 0)	data_buf[2] = -data_buf[2];
			// data_buf[2] = (data_buf[2]+1)*M_PI;
			// for(int i = 0; i < msg->transforms.size(); i++){
			// 	ROS_INFO("%d, %f, %f, %f, %f, %f, %f, %f"	,msg->transforms.size()
			// 																				,msg->transforms.at(i).transform.translation.x
			// 																				,msg->transforms.at(i).transform.translation.y
			// 																				,msg->transforms.at(i).transform.translation.z
			// 																				,msg->transforms.at(i).transform.rotation.w
			// 																				,msg->transforms.at(i).transform.rotation.x
			// 																				,msg->transforms.at(i).transform.rotation.y
			// 																				,msg->transforms.at(i).transform.rotation.z);
	    // }

			//Subscribe path_pose
			data_buf[0] = msg->data[0];
			data_buf[1] = msg->data[1];
			data_buf[2] = msg->data[2];
			ROS_INFO("x : %f, y : %f, yaw : %f", data_buf[0], data_buf[1], data_buf[2]);
		}

		public: void OnUpdate(const common::UpdateInfo &)
		{
			time += 1;
			if(time >= 100){
				time = 0;
				// data[0] += data_buf[0];
				// data[1] += data_buf[1];
				// data[2] += data_buf[2];

				// ROS_INFO("%3f", data[1]);
			}

			jcX->SetJointPosition(jX, data_buf[0]);
			jcY->SetJointPosition(jY, data_buf[1]);
			jcYaw->SetJointPosition(jYaw, data_buf[2]);
		}
	};
	GZ_REGISTER_MODEL_PLUGIN(husky)
}
