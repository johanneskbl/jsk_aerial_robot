#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <mutex>
#include <geometry_msgs/WrenchStamped.h>
#include <string>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <takasako_sps/PowerInfo.h>
#include <spinal/ESCTelemetry.h>
#include <spinal/ESCTelemetryArray.h>
#include <spinal/PwmTest.h>

namespace Mode
{
  enum {STEP = 0, ONESHOT = 1,};
};

class MotorTest
{
public:
  MotorTest(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nhp_(nh_private), start_flag_(false), pwm_value_(0), once_flag_(true)
  {
    nhp_.param("test_mode", test_mode_, (int)Mode::STEP);
    ROS_WARN("test mode: %d", test_mode_);
    nhp_.param("run_duration", run_duration_, 4.0);
    nhp_.param("pwm_incremental_value", pwm_incremental_value_, 50);
    nhp_.param("min_pwm_value", min_pwm_value_, 1100);
    nhp_.param("max_pwm_value", max_pwm_value_, 1950);

    nhp_.param("stop_pwm_value", stop_pwm_value_, 1000);
    nhp_.param("pwm_range", pwm_range_, 2000.0);

    /* one-shot mode */
    nhp_.param("raise_duration", raise_duration_, 1.0);
    nhp_.param("brake_duration", brake_duration_, 1.0);

    std::string topic_name;
    nhp_.param("force_sensor_sub_name", topic_name, std::string("forces"));
    force_snesor_sub_ = nh_.subscribe(topic_name, 1, &MotorTest::forceSensorCallback, this, ros::TransportHints().tcpNoDelay());
    nhp_.param("power_info_sub_name", topic_name, std::string("power_info"));
    power_info_sub_ = nh_.subscribe(topic_name, 1, &MotorTest::powerInfoCallback, this, ros::TransportHints().tcpNoDelay());
    nhp_.param("motor_pwm_sub_name", topic_name, std::string("power_pwm"));
    motor_pwm_pub_ = nh_.advertise<spinal::PwmTest>(topic_name,1);

    start_cmd_sub_ =  nh_.subscribe("start_log_cmd", 1,  &MotorTest::startCallback, this, ros::TransportHints().tcpNoDelay());
    sps_on_pub_ = nh.advertise<std_msgs::Empty>("/power_on_cmd", 1);

    /* dshot telemetry */
    nhp_.param("has_dshot_telemetry", has_dshot_telemetry_, false);

    if (has_dshot_telemetry_)
    {
      esc_telem_sub_ = nh_.subscribe("/esc_telem", 1, &MotorTest::escTelemCallback, this, ros::TransportHints().tcpNoDelay());
    }

    nhp_.param("rpm_mode", rpm_mode_, false);
    ROS_WARN("rpm mode: %d", rpm_mode_);
    if (rpm_mode_ && !has_dshot_telemetry_)
      ROS_WARN("rpm mode without dshot telemetry: the log contains no measured RPM to identify the rotor with");

    /* motor index, as in spinal/PwmTest: motor 0 is the DShot output M1 (esc_telemetry_1), ..., motor 3 is M4 (esc_telemetry_4).
       -1: commands all motors at once (simultaneous pwm test mode of spinal) and records the ESC telemetry of motor 0.
       0, 1, ...: commands only this motor (individual pwm test mode of spinal, the others stay off) and records its ESC telemetry */
    nhp_.param("motor_index", motor_index_, -1);
    ROS_WARN("motor index: %d", motor_index_);
    if (motor_index_ < -1 || motor_index_ >= SPINAL_MAX_MOTOR_NUMBER ||
        (has_dshot_telemetry_ && motor_index_ >= DSHOT_MOTOR_NUMBER))
      {
        ROS_ERROR("Invalid motor index %d: -1 (all motors) or 0 to %d (0 to %d with dshot telemetry)",
                  motor_index_, SPINAL_MAX_MOTOR_NUMBER - 1, DSHOT_MOTOR_NUMBER - 1);
        ros::shutdown();
        return;
      }

    /* ramp: limits the rate of change of the commanded pwm value, also when the motor starts from and stops at
       stop_pwm_value. In rpm mode, the pwm value commands a target RPM, e.g. 100 /s = 1240 RPM/s for an RPM window of
       900 / 13300 RPM. 0 disables the ramp: the command jumps */
    nhp_.param("pwm_ramp_rate", pwm_ramp_rate_, 0.0);
    ROS_WARN("pwm ramp rate: %f", pwm_ramp_rate_);
    cmd_pwm_value_ = stop_pwm_value_; // the motor is at rest when the test starts

    ROS_WARN("run: %f, raise: %f, brake: %f", run_duration_, raise_duration_, brake_duration_);

    calib_client_ = nh_.serviceClient<std_srvs::Empty>("/cfs_sensor_calib");

    /* rpm mode: wait for the force sensor node, a call before it advertises the service fails and leaves the sensor uncalibrated */
    if (rpm_mode_ && !calib_client_.waitForExistence(ros::Duration(10.0)))
      ROS_ERROR("Service /cfs_sensor_calib is not available");

    if (calibrateForceSensor())
      {
        ROS_INFO("done force sensor init calib");
      }
    else
      {
        ROS_ERROR("Failed to call service /cfs_sensor_calib");
      }

    sps_on_pub_.publish(std_msgs::Empty());

    pwm_timer_ = nhp_.createTimer(ros::Duration(PWM_TIMER_PERIOD), &MotorTest::pwmFunc,this);
  }

  ~MotorTest()
  {
  }

private:
  static constexpr double PWM_TIMER_PERIOD = 1.0 / 50; // [s] period of pwmFunc(), which also advances the ramp
  static constexpr int SPINAL_MAX_MOTOR_NUMBER = 10;   // MAX_MOTOR_NUMBER of spinal (attitude_control.h)
  static constexpr int DSHOT_MOTOR_NUMBER = 4;         // DSHOT_MOTOR_NUMBER of spinal (dshot.h): motors 0-3 report ESC telemetry

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber force_snesor_sub_;
  ros::Subscriber power_info_sub_;
  ros::Subscriber start_cmd_sub_;
  ros::Subscriber esc_telem_sub_;
  ros::Publisher motor_pwm_pub_;
  ros::Publisher sps_on_pub_;
  ros::ServiceClient calib_client_;

  ros::Timer  pwm_timer_;

  uint16_t pwm_value_;

  int test_mode_;
  bool start_flag_;
  bool once_flag_;
  double run_duration_;
  double raise_duration_, brake_duration_; // for one-shot mode
  int pwm_incremental_value_;
  int stop_pwm_value_, min_pwm_value_, max_pwm_value_;
  double pwm_range_;
  bool rpm_mode_; // the ESC runs closed-loop RPM control
  int motor_index_; // -1: all motors, else the only commanded and recorded motor, see the constructor

  /* ramp of the commanded pwm value, see the constructor and updateRamp() */
  double pwm_ramp_rate_;            // [pwm value / s], 0: no ramp
  double cmd_pwm_value_;            // pwm value commanded last
  double target_pwm_value_ = 0.0;   // pwm value at the end of the ramp
  bool ramping_ = false;
  bool stop_after_ramp_ = false;    // stop the motor when the ramp reaches target_pwm_value_
  ros::Time ramp_start_time_;

  /* guards the members shared between the callbacks, since the AsyncSpinner runs them in parallel threads */
  std::mutex mutex_;

  float currency_ = 0.0;
  ros::Time init_time_;

  std::ofstream ofs_;

  /* dshot measurement */
  bool has_dshot_telemetry_;
  uint32_t rpm_ = 0;
  int temperature_ = 0;
  float voltage_ = 0.0;

  /* Zero the force sensor. The call blocks while the sensor node averages (~1 s), the rotor must be at rest meanwhile */
  bool calibrateForceSensor()
  {
    std_srvs::Empty srv;
    return calib_client_.call(srv);
  }

  /* Command a pwm value to all motors (simultaneous pwm test mode of spinal) or to motor_index only */
  void publishPwm(double pwm_value)
  {
    spinal::PwmTest cmd_msg;
    if (motor_index_ >= 0)
      cmd_msg.motor_index.push_back(motor_index_);
    cmd_msg.pwms.push_back(pwm_value / pwm_range_);
    motor_pwm_pub_.publish(cmd_msg);
  }

  /* Stop the motor after each one-shot step and at the end of the test */
  void stopMotor()
  {
    /* rpm mode: stop_pwm_value (1000 -> duty 0.5 -> DShot 48) commands the min RPM of the ESC, so the rotor would keep
       spinning. An empty message instead makes spinal leave the pwm test mode, after which it sends DShot 0 (motor off),
       provided that the flight control has not started the motors, which is the case on the test bench.
       The next non-empty message enters the pwm test mode again. */
    if (rpm_mode_)
      motor_pwm_pub_.publish(spinal::PwmTest());
    else
      publishPwm(stop_pwm_value_);
    cmd_pwm_value_ = stop_pwm_value_;
  }

  /* Command target_pwm_value, and stop the motor once it is reached if stop is true.
     With pwm_ramp_rate > 0, pwmFunc() ramps the command there (see updateRamp()), otherwise it jumps right away */
  void rampTo(double target_pwm_value, bool stop)
  {
    target_pwm_value_ = target_pwm_value;
    stop_after_ramp_ = stop;
    if (pwm_ramp_rate_ > 0 && cmd_pwm_value_ != target_pwm_value_)
      {
        ramping_ = true;
        ramp_start_time_ = ros::Time::now();
        return;
      }

    if (stop)
      {
        stopMotor();
      }
    else
      {
        cmd_pwm_value_ = target_pwm_value_;
        publishPwm(cmd_pwm_value_);
      }
  }

  /* Advance the ramp by one period of pwmFunc(), at most pwm_ramp_rate * PWM_TIMER_PERIOD. A late call of pwmFunc()
     makes the ramp slower, never steeper. Returns true while the ramp runs */
  bool updateRamp()
  {
    if (!ramping_) return false;

    const double max_change = pwm_ramp_rate_ * PWM_TIMER_PERIOD;
    if (std::fabs(target_pwm_value_ - cmd_pwm_value_) <= max_change)
      cmd_pwm_value_ = target_pwm_value_;
    else
      cmd_pwm_value_ += std::copysign(max_change, target_pwm_value_ - cmd_pwm_value_);
    publishPwm(cmd_pwm_value_);

    if (cmd_pwm_value_ != target_pwm_value_) return true;

    /* The ramp has reached its target. The phases of a step (raise, valid, brake) do not count the time of the ramps:
       shift their time origin by the duration of the ramp, so that they start or resume only now */
    ramping_ = false;
    init_time_ += ros::Time::now() - ramp_start_time_;
    if (stop_after_ramp_)
      {
        stop_after_ramp_ = false;
        stopMotor();
      }
    return false;
  }

  void startCallback(const std_msgs::EmptyConstPtr & msg)
  {
    /* rpm mode: zero the force sensor right before the first step, as before each following step. The rotor must be at
       rest, i.e. spinal outside the pwm test mode (DShot 0): after its boot or after an empty message on the pwm topic */
    if (rpm_mode_)
      {
        if (calibrateForceSensor())
          {
            ROS_INFO("done force sensor calib");
          }
        else
          {
            ROS_ERROR("Failed to call service /cfs_sensor_calib, do not start pwm test");
            return;
          }
      }

    std::lock_guard<std::mutex> lock(mutex_);

    /* a single tested motor is part of the file name, e.g. motor_test_motor2_<time>.txt */
    std::string motor_name = (motor_index_ >= 0) ? std::string("motor") + std::to_string(motor_index_) + std::string("_") : std::string("");
    std::string file_name  = std::string("motor_test_") + motor_name + std::to_string((int)ros::Time::now().toSec()) + std::string(".txt");
    ofs_.open(file_name, std::ios::out);

    pwm_value_ = min_pwm_value_;

    rampTo(pwm_value_, false);
    init_time_ = ros::Time::now();
    ROS_INFO("start pwm test");
    start_flag_ = true;
  }

  void escTelemCallback(const spinal::ESCTelemetryArrayConstPtr & msg)
  {
    /* the ESC of the tested motor: motor_index, or motor 0 if all motors are tested */
    const spinal::ESCTelemetry* esc_telem;
    switch (motor_index_)
      {
      case 1: esc_telem = &msg->esc_telemetry_2; break;
      case 2: esc_telem = &msg->esc_telemetry_3; break;
      case 3: esc_telem = &msg->esc_telemetry_4; break;
      default: esc_telem = &msg->esc_telemetry_1; break; // motor 0 or all motors (-1)
      }

    std::lock_guard<std::mutex> lock(mutex_);
    rpm_ = esc_telem->rpm;
    temperature_ = esc_telem->temperature;
    voltage_ = (float)(esc_telem->voltage) / 100;
  }

  void powerInfoCallback(const takasako_sps::PowerInfoConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    currency_ = msg->currency;
  }

  void forceSensorCallback(const geometry_msgs::WrenchStampedConstPtr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if(!start_flag_) return;

    double force_norm = sqrt(msg->wrench.force.x * msg->wrench.force.x +
                             msg->wrench.force.y * msg->wrench.force.y +
                             msg->wrench.force.z * msg->wrench.force.z);
    if (has_dshot_telemetry_)
    {
      ofs_ << pwm_value_ << " "
           << msg->wrench.force.x << " "
           << msg->wrench.force.y << " "
           << msg->wrench.force.z << " "
           << force_norm << " "
           << msg->wrench.torque.x << " "
           << msg->wrench.torque.y << " "
           << msg->wrench.torque.z << " "
           << currency_ << " "
           << rpm_ << " "
           << temperature_ << " "
           << voltage_;
    }
    else
    {
      ofs_ << pwm_value_ << " "
           << msg->wrench.force.x << " "
           << msg->wrench.force.y << " "
           << msg->wrench.force.z << " "
           << force_norm << " "
           << msg->wrench.torque.x << " "
           << msg->wrench.torque.y << " "
           << msg->wrench.torque.z << " "
           << currency_;
    }


    if(test_mode_ == Mode::ONESHOT)
      {
        /* the command ramps (pwm_ramp_rate > 0) to the step or down to the stop, the time of the phases is paused */
        if(ramping_)
          {
            ofs_ << " " << std::string("ramp");
          }
        else if(ros::Time::now().toSec() - init_time_.toSec() < raise_duration_)
          {
            ofs_ << " " << std::string("raise");
          }
        else
          {
            if(ros::Time::now().toSec() - init_time_.toSec() < raise_duration_ + run_duration_)
              {
                ofs_ << " " << std::string("valid");
              }
            else
              {
                ofs_ << " " << std::string("brake");
              }
          }
      }

    /* rpm mode: the time stamp of the force measurement [s] as the last column. The force sensor node does not publish
       at a fixed rate, so the row number is no time base (e.g. to interpolate the zero offset of the force sensor).
       std::to_string keeps the microseconds without changing the format of the stream for the other columns. */
    if (rpm_mode_)
      ofs_ << " " << std::to_string(msg->header.stamp.toSec());

    ofs_ << std::endl;
  }

  void pwmFunc(const ros::TimerEvent & e)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    /* The phases of a step wait while the command ramps. A ramp down at the end of the test continues after it */
    if(updateRamp()) return;

    if(!start_flag_) return;

    if(ros::Time::now().toSec() - init_time_.toSec() > run_duration_)
      {
        if(test_mode_ == Mode::STEP)
          {
            pwm_value_ += pwm_incremental_value_;
            init_time_ = ros::Time::now();
          }
        else if(test_mode_ == Mode::ONESHOT)
          {
            if(ros::Time::now().toSec() - init_time_.toSec() > run_duration_ + raise_duration_)
              {
                if(once_flag_)
                  {
                    rampTo(stop_pwm_value_, true);
                    once_flag_ = false;
                    ROS_WARN("STOP");
                    return;
                  }
              }
            else
              {
                return;
              }

            if(ros::Time::now().toSec() - init_time_.toSec() > run_duration_ + raise_duration_ + brake_duration_)
              {
                /* need to wait 1s for calibration. Unlocked meanwhile, so that a force measurement that arrived before the
                   calibration is still logged with the state of this step (pwmFunc() itself never runs twice at once) */
                lock.unlock();
                bool calibrated = calibrateForceSensor();
                lock.lock();
                if (calibrated)
                  {
                    ROS_INFO("done force sensor calib");
                    once_flag_ = true;
                    pwm_value_ += pwm_incremental_value_;
                    init_time_ = ros::Time::now();
                  }
                else
                  {
                    ROS_ERROR("Failed to call service /cfs_sensor_calib");
                    pwm_value_ = max_pwm_value_ + pwm_incremental_value_;
                  }
              }
          }

        if(pwm_value_ > max_pwm_value_)
          {
            start_flag_ = false;
            rampTo(stop_pwm_value_, true);

            ROS_WARN("finish pwm test");
            ofs_ << "done" << std::endl;
            ofs_.close();

            return;
          }

        if(once_flag_)
          {
            ROS_INFO("target_pwm: %d", pwm_value_);
            rampTo(pwm_value_, false);
          }
      }
  }

};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "force_sensor_log");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  MotorTest*  logNode = new MotorTest(nh, nh_private);
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  delete logNode;
  return 0;
}
