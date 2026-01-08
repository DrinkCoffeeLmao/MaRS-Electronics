// #include "ros/ros.h"
// #include "std_msgs/Int16.h"
// #include "sensor_msgs/Joy.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <algorithm>  
#include <memory>

int drive_speed = 0; 
int manipulator_pwm = 0;
int drive_direction = 0; 
int manipulator_choice = 0;
int individual_control = 0;
int steer_msg=0;

class JoyControl : public rclcpp::Node {
public:
   JoyControl() : Node("joy_control"){
       //ros::NodeHandle nh;
       // subscribe to joystick inputs
       //joy_subscriber_ = nh.subscribe("/joy", 10, &JoyControl::joyCallback, this);
       joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyControl::joyCallback, this, std::placeholders::_1));

       // publishers for drive and manipulator
       // drive_speed_publisher_ = nh.advertise<std_msgs::Int16>("/rover/pwm", 10, true);
       drive_speed_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/rover_cmd", 10);
       //drive_direction_publisher_ = nh.advertise<std_msgs::Int16>("/rover/choice", 10, true);
       drive_direction_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/manipulator/dgfhsuyd", 10);
       //manipulator_pwm_publisher_ = nh.advertise<std_msgs::Int16>("/manipulator/pwm", 10, true);
       manipulator_pwm_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/bevel_cmd", 10);
       //manipulator_choice_publisher_ = nh.advertise<std_msgs::Int16>("/manipulator/choice", 10, true);
       manipulator_choice_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/manipulator_cmd", 10);  
      
      //publisher for steer
      //steer_publisher_ = nh.advertise<std_msgs::Int16>("/rover/steer", 10, true);
      steer_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/steer_cmd", 10);
   }


   void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
       std_msgs::msg::Int16 drive_speed_msg;
       std_msgs::msg::Int16 drive_direction_msg;
       std_msgs::msg::Int16  manipulator_pwm_msg;
       std_msgs::msg::Int16  manipulator_choice_msg;
       std_msgs::msg::Int16  steer_msg;
       //std_msgs::Int16 individual_control_msg;

      //  int drive_speed = 0;
      //  int drive_direction = 10;
      //  int manipulator_pwm = 0;
      //  int manipulator_choice = 0;
      //  int steer = 5;

       // Drive control
     //start button-increase drive's pwm and select button is decrease
       if(msg->buttons[14]==1){
        drive_direction = 0;
        drive_speed = 3;
        steer_msg.data=0;
        
        RCLCPP_INFO(this->get_logger(), "Rover: Stop");
    }
       else if (msg->buttons[0] == 1) { 
        //    drive_direction = 2; 
        drive_speed=2;
        RCLCPP_INFO(this->get_logger(), "Drive: Decrease Speed: %d", drive_speed);
        //    RCLCPP_INFO(this->get_logger(), "Drive: Backward");
       } else if (msg->buttons[4] == 1) { 
        //    drive_direction = 1;
        drive_speed=1;
         RCLCPP_INFO(this->get_logger(), "Drive: Increase Speed: %d", drive_speed);
        //    RCLCPP_INFO(this->get_logger(), "Drive: Forward");
       }

        else if (msg->buttons[1] == 1) {
           drive_speed = 4;
           RCLCPP_INFO(this->get_logger(), "Spot: Right");
       } else if (msg->buttons[3] == 1) {
            drive_speed = 5;
           RCLCPP_INFO(this->get_logger(), "Spot: Left");
       } else if (msg->buttons[10] == 1) { 
        //    drive_speed = std::max(drive_speed - 5, 0);
        //    RCLCPP_INFO(this->get_logger(), "Drive: Decrease Speed: %d", drive_speed);
        steer_msg.data=10;
         RCLCPP_INFO(this->get_logger(), "Spot Turn");
       } else if (msg->buttons[11] == 1) { 
        steer_msg.data=11;
        RCLCPP_INFO(this->get_logger(), "Spot Turn reverse");
        //    drive_speed = std::min(drive_speed + 5, 255);
        //    RCLCPP_INFO(this->get_logger(), "Drive: Increase Speed: %d", drive_speed);
       }else if ((msg->buttons[6] == 1) && (msg->buttons[7] == 1)){ //steer code starts
       		//**Spot Turn Rev
              steer_msg.data = 50;
            RCLCPP_INFO(this->get_logger(), "both wheels left"); //left
            }else if ((msg->buttons[9] == 1) && (msg->buttons[8] == 1)){
            
              steer_msg.data = 25;
            RCLCPP_INFO(this->get_logger(), "both wheels right"); //right
            
       }else if(msg->buttons[9] == 1){
       
            steer_msg.data = 2;
            RCLCPP_INFO(this->get_logger(), "right wheel right"); //right
            
            }else if(msg->buttons[7] == 1){
            //**Spot Turn
            steer_msg.data = 1;
            RCLCPP_INFO(this->get_logger(), "right wheel left"); //left
            
            }else if(msg->buttons[6] == 1){
            
              steer_msg.data = 3;
            RCLCPP_INFO(this->get_logger(), "left wheel left"); //left
            
            }else if(msg->buttons[8] == 1){
            
              steer_msg.data = 4;
            RCLCPP_INFO(this->get_logger(), "left wheel right"); //right
            }
            
                   else{
       drive_speed=0;
       steer_msg.data=0;
       
       }
       
       
if ((msg->buttons[0] == 1)||(msg->buttons[4] == 1))
    {
            if(drive_speed>0){
                drive_direction=1;
            }else if(drive_speed<0){
                drive_direction=2;

            }
    }
       // manipulator's pwm
       //if (msg->buttons[7] == 1) {
           //manipulator_pwm = std::min(manipulator_pwm + 5, 255);
           //ROS_INFO("Manipulator: Increase PWM: %d", manipulator_pwm);
       //} else if (msg->buttons[6] == 1) { 
           //manipulator_pwm = std::max(manipulator_pwm - 5, 0);
           //ROS_INFO("Manipulator: Decrease PWM: %d", manipulator_pwm);

       
       if ((msg->buttons[14] == 1)||(msg->axes[1] == 0 && msg->axes[3] == 0 && msg->axes[2] == 0 && msg->axes[2] == 0 && msg->axes[0] == 0 && msg->axes[7]== 0)) {
           manipulator_choice = 0;
           manipulator_pwm = 3;
       }
            else {
           //link 1
           if (msg->axes[1] == +1) { 
               manipulator_choice = 4;
               RCLCPP_INFO(this->get_logger(), "Manipulator: Link 1 Up");
           
           } else if(msg->axes[1] ==-1) { 
               manipulator_choice = 3;
               RCLCPP_INFO(this->get_logger(), "Manipulator: Link 1 Down");
               
           } 
           //link 2
           if (msg->axes[3] == +1) {
               manipulator_choice = 6;
               RCLCPP_INFO(this->get_logger(), "Manipulator: Link 2 Up");
               
           } else if (msg->axes[3] ==-1) {
               manipulator_choice = 5;
               RCLCPP_INFO(this->get_logger(), "Manipulator: Link 2 Down");
               
           }

           //bevel rotate 
           if (msg->axes[2] == +1) {
               
               RCLCPP_INFO(this->get_logger(), "Manipulator: Bevel Rotate Left");
               manipulator_pwm = 2;
           } else if(msg->axes[2] ==-1) {
               
               RCLCPP_INFO(this->get_logger(), "Manipulator: Bevel Rotate Right"); 
               manipulator_pwm = 1;                                   
           } 

           if (msg->axes[0] == -1) {
               manipulator_choice = 8;
               RCLCPP_INFO(this->get_logger(), "Manipulator: Base Right");
              
           } else if (msg->axes[0] ==+1) {
               manipulator_choice = 9;
               RCLCPP_INFO(this->get_logger(), "Manipulator: Base Left"); 
               
           }

           //bevel up down
            if(msg->axes[7]== +1){
            
            RCLCPP_INFO(this->get_logger(), "Manipulator: Bevel_gear Up");
            manipulator_pwm =5 ;
           }
           else if(msg->axes[7]== -1){
            
            RCLCPP_INFO(this->get_logger(), "Manipulator: Bevel_gear Down");
            manipulator_pwm =4;
           }
           
           



           //else if(msg->axes[7]== +1){
            //manipulator_choice = 11;
            //RCLCPP_INFO(this->get_logger(), "Manipulator: Gripper Close");
            //manipulator_pwm = 150;
           //}
           //else if(msg->axes[7]== -1){      
            //manipulator_choice = 12;
            //RCLCPP_INFO(this->get_logger(), "Manipulator: Gripper Open");
            //manipulator_pwm = 150;
           //}
           }
        if (msg->buttons[1] == 1) {
           drive_speed = 4;
           RCLCPP_INFO(this->get_logger(), "Spot: Right");
       } else if (msg->buttons[3] == 1) {
            drive_speed = 5;
           RCLCPP_INFO(this->get_logger(), "Spot: Left");
       } else if (msg->buttons[10] == 1) { 
        //    drive_speed = std::max(drive_speed - 5, 0);
        //    RCLCPP_INFO(this->get_logger(), "Drive: Decrease Speed: %d", drive_speed);
        steer_msg.data=10;
         RCLCPP_INFO(this->get_logger(), "Spot Turn");
       } else if (msg->buttons[11] == 1) { 
        steer_msg.data=11;
        RCLCPP_INFO(this->get_logger(), "Spot Turn reverse");
        //    drive_speed = std::min(drive_speed + 5, 255);
        //    RCLCPP_INFO(this->get_logger(), "Drive: Increase Speed: %d", drive_speed);
       }
	//}
         /* while(ros::ok()){
            std::cout<<"individual control "<<std::endl;

            std::cin>> individual_control;
          }*/

          // if(individual_control == 1){
          //   ROS_INFO("FL:  turn");
          // }else if(individual_control == 2){
          //   ROS_INFO("FL:  turn reverse");
          // }else if(individual_control == 3){
          //   ROS_INFO("FR:  turn");
          // }else if(individual_control == 4){
          //   ROS_INFO("FR:  turn reverse");
          // }else if(individual_control == 5){
          //   ROS_INFO("RL:  turn");
          // }else if(individual_control == 6){
          //   ROS_INFO("RL:  turn reverse");
          // }else if(individual_control == 7){
          //   ROS_INFO("RR:  turn");
       // publish drive msgs
       drive_speed_msg.data = abs(drive_speed);
       drive_direction_msg.data = drive_direction;
       drive_speed_publisher_->publish(drive_speed_msg);
       drive_direction_publisher_->publish(drive_direction_msg);
       // publish manipulator msgs
       manipulator_pwm_msg.data = manipulator_pwm;
       manipulator_choice_msg.data = manipulator_choice;
       manipulator_pwm_publisher_->publish(manipulator_pwm_msg);
       manipulator_choice_publisher_->publish(manipulator_choice_msg);
       steer_publisher_->publish(steer_msg);


   }


private:
  // ros::Subscriber joy_subscriber_;
   rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_; 
   //ros::Publisher drive_speed_publisher_;
   rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr drive_speed_publisher_;
   //ros::Publisher drive_direction_publisher_;
   rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr drive_direction_publisher_;
   //ros::Publisher manipulator_pwm_publisher_;
   rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr manipulator_pwm_publisher_; 
   //ros::Publisher manipulator_choice_publisher_;
   rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr manipulator_choice_publisher_;
   //ros::Publisher steer_publisher_;
   rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr steer_publisher_;
};


int main(int argc, char **argv) 
{
  // ros::init(argc, argv, "joy_control_node");
   rclcpp::init(argc, argv);
   //JoyControl joy_control;
   auto node = std::make_shared<JoyControl>();
   RCLCPP_INFO(node->get_logger(), "Joystick rover and manipulator control node initialized.");
   //ros::spin();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
