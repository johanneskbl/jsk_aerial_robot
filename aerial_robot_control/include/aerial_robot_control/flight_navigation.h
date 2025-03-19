#pragma once

#include <angles/angles.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Joy.h>
#include "aerial_robot_estimation/state_estimation.h"
#include "aerial_robot_estimation/sensor/base_plugin.h"
#include "aerial_robot_estimation/sensor/gps.h"
#include "aerial_robot_msgs/FlightNav.h"
#include "spinal/FlightConfigCmd.h"

namespace aerial_robot_navigation
{
    /* Navigation state */
    enum flight_state
    {
        ARM_OFF_STATE,
        START_STATE,
        ARM_ON_STATE,
        TAKEOFF_STATE,
        LAND_STATE,
        HOVER_STATE,
        STOP_STATE
    };
    
    /* Control frame */
    enum control_frame
    {
        WORLD_FRAME,    // Global frame, e.g. ENU, mocap
        LOCAL_FRAME     // Head frame which is identical with imu head direction
    };

    /* Control mode */
    enum control_mode
    {
        POS_CONTROL_MODE,
        VEL_CONTROL_MODE,
        ACC_CONTROL_MODE
    };

    class BaseNavigator
    {
        public:
            //// Functions ////
            BaseNavigator();
            virtual ~BaseNavigator() = default;

            // Initialization
            virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator);
                                    
            // Update
            virtual void update();
            
            // Publish TODO not used!
            //void tfPublish();
            
            // Setters
            /* Estimation */
            void setEstimateMode(uint8_t estimate_mode) { estimate_mode_ = estimate_mode; }

            /* Navigation */
            inline void setNaviState(const uint8_t state) { navi_state_ = state; }
            
            /* Target state */
            inline void setTargetPosX(float value) { target_pos_.setX(value); }
            inline void setTargetPosY(float value) { target_pos_.setY(value); }
            inline void setTargetPosZ(float value) { target_pos_.setZ(value); }

            inline void setTargetVelX(float value) { target_vel_.setX(value); }
            inline void setTargetVelY(float value) { target_vel_.setY(value); }
            inline void setTargetVelZ(float value) { target_vel_.setZ(value); }

            inline void setTargetAccX(float value) { target_acc_.setX(value); }
            inline void setTargetAccY(float value) { target_acc_.setY(value); }
            inline void setTargetAccZ(float value) { target_acc_.setZ(value); }

            inline void setTargetRPY(tf::Vector3 value) { target_rpy_ = value; }
            inline void setTargetRoll(float value)      { target_rpy_.setX(value); }
            inline void setTargetPitch(float value)     { target_rpy_.setY(value); }
            inline void setTargetYaw(float value)       { target_rpy_.setZ(value); }

            inline void setTargetOmega(tf::Vector3 value) { target_omega_ = value; }
            inline void setTargetOmegaX(float value)      { target_omega_.setX(value); }
            inline void setTargetOmegaY(float value)      { target_omega_.setY(value); }
            inline void setTargetOmegaZ(float value)      { target_omega_.setZ(value); }
            
            /* Control */
            inline void setControlFrame(uint8_t frame_type) { control_frame_ = frame_type; }
            inline void setXyControlMode(uint8_t mode) { xy_control_mode_ = mode; }
            
            /* Teleoperation */
            inline void setTeleopFlag(bool teleop_flag) { teleop_flag_ = teleop_flag; }
            
            // Getters
            /* ROS Communication */
            ros::Publisher& getFlightConfigPublisher() { return flight_config_pub_; }
            
            /* Estimation */
            uint8_t getEstimateMode() { return estimate_mode_; }

            /* Navigation */
            inline uint8_t getNaviState() { return navi_state_; }
            
            /* Target state */
            inline tf::Vector3 getTargetPos()   { return target_pos_; }
            inline tf::Vector3 getTargetVel()   { return target_vel_; }
            inline tf::Vector3 getTargetAcc()   { return target_acc_; }
            inline tf::Vector3 getTargetRPY()   { return target_rpy_; }
            inline tf::Vector3 getTargetOmega() { return target_omega_; }
            inline void addTargetPosZ(float value) { target_pos_ += tf::Vector3(0, 0, value); }

            /* Control */
            inline uint8_t getControlFrame() { return (uint8_t) control_frame_; }
            inline uint8_t getXyControlMode() { return (uint8_t) xy_control_mode_; }
            
            /* Flight state (Takeoff & Landing) */
            inline bool getXyVelModePosCtrlTakeoff() { return xy_vel_mode_pos_ctrl_takeoff_; }
            inline bool getForceLandingFlag() { return force_landing_flag_; }
            inline double getForceLandingStartTime() { return force_landing_start_time_.toSec(); }
            
            /* Teleoperation */
            inline bool getTeleopFlag() { return teleop_flag_; }
            
            //// Constants ////
            /* Control */
            static constexpr uint8_t POS_CONTROL_COMMAND = 0;
            static constexpr uint8_t VEL_CONTROL_COMMAND = 1;

            /* Abnormal state */
            static constexpr uint8_t LOW_BATTERY_STATE = 0x10;
            static constexpr uint8_t FORCE_LANDING_STATE = 0x11;

            /* Battery check */
            static constexpr float VOLTAGE_100P = 4.2;
            static constexpr float VOLTAGE_90P = 4.085;
            static constexpr float VOLTAGE_80P = 3.999;
            static constexpr float VOLTAGE_70P = 3.936;
            static constexpr float VOLTAGE_60P = 3.883;
            static constexpr float VOLTAGE_50P = 3.839;
            static constexpr float VOLTAGE_40P = 3.812;
            static constexpr float VOLTAGE_30P = 3.791;
            static constexpr float VOLTAGE_20P = 3.747;
            static constexpr float VOLTAGE_10P = 3.683;
            static constexpr float VOLTAGE_0P = 3.209;

            /* Joystick (Playstation Dualschock 3) */
            static constexpr int PS3_BUTTONS = 17;
            static constexpr int PS3_BUTTON_SELECT = 0;
            static constexpr int PS3_BUTTON_STICK_LEFT = 1;
            static constexpr int PS3_BUTTON_STICK_RIGHT = 2;
            static constexpr int PS3_BUTTON_START = 3;
            static constexpr int PS3_BUTTON_CROSS_UP = 4;
            static constexpr int PS3_BUTTON_CROSS_RIGHT = 5;
            static constexpr int PS3_BUTTON_CROSS_DOWN = 6;
            static constexpr int PS3_BUTTON_CROSS_LEFT = 7;
            static constexpr int PS3_BUTTON_REAR_LEFT_2 = 8;
            static constexpr int PS3_BUTTON_REAR_RIGHT_2 = 9;
            static constexpr int PS3_BUTTON_REAR_LEFT_1 = 10;
            static constexpr int PS3_BUTTON_REAR_RIGHT_1 = 11;
            static constexpr int PS3_BUTTON_ACTION_TRIANGLE = 12;
            static constexpr int PS3_BUTTON_ACTION_CIRCLE = 13;
            static constexpr int PS3_BUTTON_ACTION_CROSS = 14;
            static constexpr int PS3_BUTTON_ACTION_SQUARE = 15;
            static constexpr int PS3_BUTTON_PAIRING = 16;
            static constexpr int PS3_AXES = 29;
            static constexpr int PS3_AXIS_STICK_LEFT_LEFTWARDS = 0;
            static constexpr int PS3_AXIS_STICK_LEFT_UPWARDS = 1;
            static constexpr int PS3_AXIS_STICK_RIGHT_LEFTWARDS = 2;
            static constexpr int PS3_AXIS_STICK_RIGHT_UPWARDS = 3;
            static constexpr int PS3_AXIS_BUTTON_CROSS_UP = 4;
            static constexpr int PS3_AXIS_BUTTON_CROSS_RIGHT = 5;
            static constexpr int PS3_AXIS_BUTTON_CROSS_DOWN = 6;
            static constexpr int PS3_AXIS_BUTTON_CROSS_LEFT = 7;
            static constexpr int PS3_AXIS_BUTTON_REAR_LEFT_2 = 8;
            static constexpr int PS3_AXIS_BUTTON_REAR_RIGHT_2 = 9;
            static constexpr int PS3_AXIS_BUTTON_REAR_LEFT_1 = 10;
            static constexpr int PS3_AXIS_BUTTON_REAR_RIGHT_1 = 11;
            static constexpr int PS3_AXIS_BUTTON_ACTION_TRIANGLE = 12;
            static constexpr int PS3_AXIS_BUTTON_ACTION_CIRCLE = 13;
            static constexpr int PS3_AXIS_BUTTON_ACTION_CROSS = 14;
            static constexpr int PS3_AXIS_BUTTON_ACTION_SQUARE = 15;
            static constexpr int PS3_AXIS_ACCELEROMETER_LEFT = 16;
            static constexpr int PS3_AXIS_ACCELEROMETER_FORWARD = 17;
            static constexpr int PS3_AXIS_ACCELEROMETER_UP = 18;
            static constexpr int PS3_AXIS_GYRO_YAW = 19;

            /* Joystick (Playstation Dualschock 4) */
            static constexpr int PS4_BUTTONS = 14;
            static constexpr int PS4_BUTTON_ACTION_SQUARE = 0;
            static constexpr int PS4_BUTTON_ACTION_CROSS = 1;
            static constexpr int PS4_BUTTON_ACTION_CIRCLE = 2;
            static constexpr int PS4_BUTTON_ACTION_TRIANGLE = 3;
            static constexpr int PS4_BUTTON_REAR_LEFT_1 = 4;
            static constexpr int PS4_BUTTON_REAR_RIGHT_1 = 5;
            static constexpr int PS4_BUTTON_REAR_LEFT_2 = 6;
            static constexpr int PS4_BUTTON_REAR_RIGHT_2 = 7;
            static constexpr int PS4_BUTTON_SHARE = 8;
            static constexpr int PS4_BUTTON_OPTIONS = 9;
            static constexpr int PS4_BUTTON_STICK_LEFT = 10;
            static constexpr int PS4_BUTTON_STICK_RIGHT = 11;
            static constexpr int PS4_BUTTON_PAIRING = 12;
            static constexpr int PS4_BUTTON_TOUCHPAD = 13;
            static constexpr int PS4_AXES = 14;
            static constexpr int PS4_AXIS_STICK_LEFT_LEFTWARDS = 0;
            static constexpr int PS4_AXIS_STICK_LEFT_UPWARDS = 1;
            static constexpr int PS4_AXIS_STICK_RIGHT_LEFTWARDS = 2;
            static constexpr int PS4_AXIS_BUTTON_REAR_LEFT_2 = 3;   // neutral=+1, full accel=-1
            static constexpr int PS4_AXIS_BUTTON_REAR_RIGHT_2 = 4;  // neutral=+1, full accel=-1
            static constexpr int PS4_AXIS_STICK_RIGHT_UPWARDS = 5;
            static constexpr int PS4_AXIS_ACCELEROMETER_LEFT = 6;
            static constexpr int PS4_AXIS_ACCELEROMETER_FORWARD = 7;
            static constexpr int PS4_AXIS_ACCELEROMETER_UP = 8;
            static constexpr int PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT = 9;  // left = +1, right= -1
            static constexpr int PS4_AXIS_BUTTON_CROSS_UP_DOWN = 10;    // up = +1, down= -1
            static constexpr int PS4_AXIS_GYRO_ROLL = 11;
            static constexpr int PS4_AXIS_GYRO_YAW = 12;
            static constexpr int PS4_AXIS_GYRO_PITCH = 13;

            static const sensor_msgs::Joy ps4joyToPs3joyConvert(const sensor_msgs::Joy& ps4_joy_msg);

        protected:
            //// Variables ////
            ros::NodeHandle nh_;
            ros::NodeHandle nhp_;

            ros::Publisher flight_config_pub_;
            ros::Publisher power_info_pub_;
            ros::Publisher flight_state_pub_;

            ros::Subscriber navi_sub_;
            ros::Subscriber battery_sub_;
            ros::Subscriber flight_status_ack_sub_;
            ros::Subscriber takeoff_sub_;
            ros::Subscriber land_sub_;
            ros::Subscriber start_sub_;
            ros::Subscriber halt_sub_;
            ros::Subscriber force_landing_sub_;
            ros::Subscriber ctrl_mode_sub_;
            ros::Subscriber joy_stick_sub_;
            ros::Subscriber flight_nav_sub_;
            ros::Subscriber stop_teleop_sub_;

            boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;
            
            bool param_verbose_;
            
            /* Estimation */
            int estimate_mode_;
            
            /* Navigation */
            bool start_able_;
            uint8_t navi_state_;
            
            /* Auto velocity navigation */
            bool vel_based_waypoint_;
            double nav_vel_limit_;      // the vel limitation
            double vel_nav_threshold_;  // the range (board) to siwtch between vel_nav and pos_nav
            double vel_nav_gain_;
            
            /* GPS Waypoint */
            bool gps_waypoint_;
            geographic_msgs::GeoPoint target_wp_;
            double gps_waypoint_time_;
            double gps_waypoint_check_du_;
            double gps_waypoint_threshold_;
            
            /* Target state */
            double max_target_tilt_angle_;
            double max_target_vel_;
            double max_target_yaw_rate_;
            tf::Vector3 target_pos_, target_vel_, target_acc_;
            tf::Vector3 target_rpy_, target_omega_;

            /* Control */
            int control_frame_;
            int xy_control_mode_;
            int prev_xy_control_mode_;
            bool pos_control_flag_;
            bool xy_control_flag_;
            bool z_control_flag_;
            bool yaw_control_flag_;
            bool vel_control_flag_;
            
            /* Flight state (Takeoff & Landing) */
            double takeoff_height_;
            bool xy_vel_mode_pos_ctrl_takeoff_;
            double convergent_start_time_;
            double convergent_duration_;
            double z_convergent_thresh_;
            double xy_convergent_thresh_;
            bool force_att_control_flag_;
            bool force_landing_flag_;
            bool force_landing_auto_stop_flag_;
            double force_landing_to_halt_du_;
            ros::Time force_landing_start_time_;
            
            /* Teleoperation */
            std::string teleop_local_frame_;
            bool lock_teleop_;
            
            /* Teleoperation */
            bool teleop_flag_;
            
            /* Joystick */
            bool joy_udp_;
            bool check_joy_stick_heart_beat_;
            bool joy_stick_heart_beat_;
            double joy_stick_heart_beat_du_;
            double joy_stick_prev_time_;
            double joy_z_deadzone_;
            double joy_yaw_deadzone_;
            double joy_target_z_interval_;
            double joy_target_vel_interval_;
            
            virtual void joyStickControl(const sensor_msgs::JoyConstPtr& joy_msg);
            
            /* Battery information */
            double low_voltage_thre_;
            bool low_voltage_flag_;
            int bat_cell_;
            double bat_resistance_;
            double bat_resistance_voltage_rate_;
            double hovering_current_;
            
            //// Functions ////
            /* ROS Communication */
            virtual void rosParamInit();
            
            /* Estimation */
            virtual void reset()
            {
                estimator_->setSensorFusionFlag(false);
                estimator_->setLandingMode(false);
                estimator_->setLandedFlag(false);
                estimator_->setFlyingFlag(false);
            }
            
            /* Navigation */
            virtual void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg);
            virtual void halt() {}
            
            /* Target state */
            void setTargetXyFromCurrentState()
            {
                setXyControlMode(POS_CONTROL_MODE);
                tf::Vector3 pos_cog = estimator_->getPos(Frame::COG, estimate_mode_);
                target_pos_.setX(pos_cog.x());
                target_pos_.setY(pos_cog.y());

                // Set velocity to zero
                target_vel_.setX(0);
                target_vel_.setY(0);

                // Set acceleration to zero
                setTargetAccX(0);
                setTargetAccY(0);
            }
            
            void setTargetZFromCurrentState()
            {
                target_pos_.setZ(estimator_->getPos(Frame::COG, estimate_mode_).z());

                // Set velocity to zero
                target_vel_.setZ(0);
            }

            void setTargetYawFromCurrentState()
            {
                target_rpy_.setZ(estimator_->getEuler(Frame::COG, estimate_mode_).z());

                // Set velocity to zero
                target_omega_.setZ(0);
            }

            /* Control */
            tf::Vector3 frameConversion(tf::Vector3 origin_val, tf::Matrix3x3 r) { return r * origin_val; }
            tf::Vector3 frameConversion(tf::Vector3 origin_val, float yaw) { return frameConversion(origin_val, tf::Matrix3x3(tf::createQuaternionFromYaw(yaw))); }
            
            // Callback
            void xyControlModeCallback(const std_msgs::Int8ConstPtr& msg)
            {
                if (getNaviState() > START_STATE)
                {
                    if (msg->data == 0)
                    {
                        setTargetXyFromCurrentState();
                        target_vel_.setValue(0, 0, 0);
                        target_acc_.setValue(0, 0, 0);
                        xy_control_mode_ = POS_CONTROL_MODE;
                        ROS_INFO("x/y position control mode");
                    }
                    if (msg->data == 1)
                    {
                        target_vel_.setValue(0, 0, 0);
                        target_acc_.setValue(0, 0, 0);
                        xy_control_mode_ = VEL_CONTROL_MODE;
                        ROS_INFO("x/y velocity control mode");
                    }
                    if (msg->data == 2)
                    {
                        xy_control_mode_ = ACC_CONTROL_MODE;
                        ROS_INFO("x/y acceleration control mode");
                    }
                }
            }
        
            /* Flight state (Takeoff & Landing) */
            void startTakeoff()
            {
                if (getNaviState() == TAKEOFF_STATE)
                return;
                
                if (getNaviState() == ARM_ON_STATE)
                {
                    setNaviState(TAKEOFF_STATE);
                    ROS_INFO("Takeoff state");
                }
            }
        
            void motorArming()
            {
                /* z(altitude) */
                /* check whether there is the fusion for the altitude */
                if (!estimator_->getStateStatus(State::Z_BASE, estimate_mode_))
                {
                    ROS_ERROR("Flight Navigation: No correct sensor fusion for z(altitude), can not fly");
                    return;
                }
                
                for (const auto& handler : estimator_->getGpsHandlers())
                {
                    if (handler->getStatus() == Status::ACTIVE)
                    {
                        ros::NodeHandle nh(nh_, "navigation");
                        nh.param("outdoor_takeoff_height", takeoff_height_, 1.2);
                        nh.param("outdorr_convergent_duration", convergent_duration_, 0.5);
                        nh.param("outdoor_xy_convergent_thresh", xy_convergent_thresh_, 0.6);
                        nh.param("outdoor_z_convergent_thresh", z_convergent_thresh_, 0.05);
                        
                        ROS_WARN_STREAM("update the navigation parameters for outdoor flight, takeoff height: "
                            << takeoff_height_ << "; outdorr_convergent_duration: " << convergent_duration_
                            << "; outdoor_xy_convergent_thresh: " << xy_convergent_thresh_
                            << "; outdoor_z_convergent_thresh: " << z_convergent_thresh_);
                            
                        break;
                    }
                }
                
                setNaviState(START_STATE);
                setTargetXyFromCurrentState();
                estimator_->setLandingHeight(estimator_->getPos(Frame::COG, estimate_mode_).z());
                setTargetPosZ(takeoff_height_);
                
                setTargetYawFromCurrentState();
                
                ROS_INFO("Start state");
            }
            
            // Callbacks
            void takeoffCallback(const std_msgs::EmptyConstPtr& msg) { startTakeoff(); }
            void startCallback(const std_msgs::EmptyConstPtr& msg) { motorArming(); }

            void flightStatusAckCallback(const std_msgs::UInt8ConstPtr& ack_msg)
            {
                if (ack_msg->data == spinal::FlightConfigCmd::ARM_OFF_CMD)
                {  //  arming off
                    ROS_INFO("STOP RES From AERIAL ROBOT");
                    setNaviState(ARM_OFF_STATE);
                }
                
                if (ack_msg->data == spinal::FlightConfigCmd::ARM_ON_CMD)
                {  //  arming on
                    ROS_INFO("START RES From AERIAL ROBOT");
                    setNaviState(ARM_ON_STATE);
                }
                
                if (ack_msg->data == spinal::FlightConfigCmd::FORCE_LANDING_CMD)
                {  //  get the first force landing message from spinal
                    ROS_INFO("FORCE LANDING MSG From AERIAL ROBOT");
                    
                    force_landing_flag_ = true;
                }
            }

            void landCallback(const std_msgs::EmptyConstPtr& msg)
            {
                if (force_att_control_flag_)
                return;
                
                if (!teleop_flag_)
                return;
                
                setNaviState(LAND_STATE);
                
                setTargetXyFromCurrentState();
                setTargetYawFromCurrentState();
                setTargetPosZ(estimator_->getLandingHeight());
                ROS_INFO("Land state");
            }
            
            void haltCallback(const std_msgs::EmptyConstPtr& msg)
            {
                if (!teleop_flag_)
                return;
                
                force_landing_flag_ = true;
                
                setNaviState(STOP_STATE);
                setTargetXyFromCurrentState();
                setTargetYawFromCurrentState();
                setTargetPosZ(estimator_->getLandingHeight());
                
                ROS_INFO("Halt state");
            }
            
            void forceLandingCallback(const std_msgs::EmptyConstPtr& msg)
            {
                spinal::FlightConfigCmd flight_config_cmd;
                flight_config_cmd.cmd = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
                flight_config_pub_.publish(flight_config_cmd);
                force_landing_flag_ = true;
                
                ROS_INFO("Force Landing state");
            }
            
            /* Teleoperation */
            // Callback
            void stopTeleopCallback(const std_msgs::UInt8ConstPtr& stop_msg)
            {
                if (stop_msg->data == 1)
                {
                ROS_WARN("stop teleop control");
                teleop_flag_ = false;
                }
                else if (stop_msg->data == 0)
                {
                ROS_WARN("start teleop control");
                teleop_flag_ = true;
                }
            }

            /* Battery information */
            void batteryCheckCallback(const std_msgs::Float32ConstPtr& msg);

            /* Helper Function */
            template <class T> void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
            {
                nh.param<T>(param_name, param, default_value);

                if (param_verbose_) ROS_INFO_STREAM("[" << nh.getNamespace() << "] " << param_name << ": " << param);
            }
    };
};
