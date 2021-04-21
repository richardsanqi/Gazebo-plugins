#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/JointController.hh>

namespace gazebo
{
  class ModelJointControler : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_parent->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, velocity plugin not loaded\n";
        return;
      }
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelJointControler::OnUpdate, this));
      
      this->old_secs =ros::Time::now().toSec();


      if (_sdf->HasElement("jointname"))
        joint_name_ori = _sdf->Get<std::string>("jointname");
        joint_name = _parent->GetJoint(joint_name_ori)->GetScopedName();
      if (_sdf->HasElement("wheel_kp"))
          this->wheel_kp = _sdf->Get<double>("wheel_kp");
      if (_sdf->HasElement("wheel_ki"))
          this->wheel_ki = _sdf->Get<double>("wheel_ki");
      if (_sdf->HasElement("wheel_kd"))
          this->wheel_kd = _sdf->Get<double>("wheel_kd");
      if (_sdf->HasElement("namespace_model"))
          this->namespace_model = _sdf->Get<std::string>("namespace_model");
      if (_sdf->HasElement("activate_pid_control"))
          this->activate_pid_control = (_sdf->Get<std::string>("activate_pid_control") == "yes");


      // Create a topic name
      std::string topic_name = "/" + this->model->GetName() + "/" + joint_name_ori + "/vel_cmd";
      
      


      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "set_wheelSpeed_rosnode",
            ros::init_options::NoSigintHandler);
      }
         
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("earthquake_rosnode"));
      

      if(this->activate_pid_control)
      {
          // Activated PID Speed Control
          const auto &jointController = this->model->GetJointController();
	        //jointController.reset (new physics::JointController(this->model));
          jointController->Reset();
          jointController->AddJoint(model->GetJoint(joint_name_ori));
          //  瞬时设定速度
          this->model->GetJoint(joint_name_ori)->SetVelocity(0,0.0);
          //通过力和速度的方式设置关节速度
          //this->model->GetJoint(joint_name_ori)->SetParam("fmax",0,0.0);
          //this->model->GetJoint(joint_name_ori)->SetParam("vel",0,0.0);
          //  PID设置速度
          //jointController->SetVelocityPID(joint_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          //jointController->SetVelocityTarget(joint_name, 0.0);
      }

      // Freq 
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            topic_name,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_joint_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ModelJointControler::QueueThread, this));
      ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());
      
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double new_secs =ros::Time::now().toSec();
      double delta = new_secs - this->old_secs;
      
      double max_delta = 0.0;
      
      if (this->freq_update != 0.0)
      {
        max_delta = 1.0 / this->freq_update;
      }
      
      if (delta > max_delta && delta != 0.0)
      {
        this->old_secs = new_secs;

	if(this->activate_pid_control)
        {
    ROS_DEBUG("Update Wheel Speed PID...");
	  const auto &jointController = this->model->GetJointController();
    //瞬时速度设置
	  this->model->GetJoint(joint_name_ori)->SetVelocity(0,this->joint_speed_magn);
    //通过力和速度的方式设置关节速度
    //this->model->GetJoint(joint_name_ori)->SetParam("fmax",0,0.1);
    //this->model->GetJoint(joint_name_ori)->SetParam("vel",0,this->joint_speed_magn);
    //通过PID控制器
    //jointController->SetVelocityTarget(joint_name, this->joint_speed_magn);
    //jointController->Update();
        }else
        {
            // Apply a small linear velocity to the model.
            ROS_DEBUG("Update Wheel Speed BASIC...");
      	    this->model->GetJoint("shoulder_joint")->SetVelocity(0, this->joint_speed_magn);
            std::cerr<<"why"<<std::endl;
        }

      }

    }
    
    
    public: void SetLeft_Joint_Speed(const double &_freq)
    {
      this->joint_speed_magn = _freq;
      ROS_WARN("1 %f", this->joint_speed_magn);
    }

    public: void OnRosMsg_joint_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetLeft_Joint_Speed(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    
 
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Time Memory
    double old_secs;
    
    // Frequency of earthquake
    double freq_update = 10.0;

    
    // Magnitude of the Oscilations
    double joint_speed_magn = 0.0;
    
    
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
    std::string topic_name;
    
    std::string namespace_model = "";
    bool activate_pid_control;

    double wheel_kp = 0.1;
    double wheel_ki = 0.0;
    double wheel_kd = 0.0;
    
    public: std::string joint_name, joint_name_ori;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelJointControler)
}