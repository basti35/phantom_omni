#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
//#define BT_EULER_DEFAULT_ZYX
//#include <bullet/LinearMath/btMatrix3x3.h>

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"
#include <pthread.h>

float prev_time;
int calibrationStyle;

struct OmniState {
  hduVector3Dd position;  //3x1 vector of position
  hduVector3Dd velocity;  //3x1 vector of velocity
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  hduVector3Dd joints;
  hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool apply_force;
  hduVector3Dd lock_pos;
  double units_ratio;
};

class PhantomROS
{
public:
  ros::NodeHandle nh_;
  ros::Publisher state_pub_,
                 pose_pub_,
                 button_pub_,
                 joint_pub_,
                 curr_vel_pub_;
  ros::Subscriber haptic_sub_;
  std::string omni_name_, ref_frame, units;

  OmniState *state_;

  void init(OmniState *s) {
    ros::param::param(std::string("~reference_frame"), ref_frame, std::string("/base_link"));
    ros::param::param(std::string("~units"), units, std::string("mm"));
    
    //Publish button state on NAME/button
    button_pub_ = nh_.advertise<omni_msgs::OmniButtonEvent>("button", 100);

    //Publish on NAME/state
    state_pub_ = nh_.advertise<omni_msgs::OmniState>("state", 1);
    
    //Subscribe to NAME/force_feedback
    haptic_sub_ = nh_.subscribe("force_feedback", 100, &PhantomROS::force_callback, this);

    //Publish on NAME/pose
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
    
    //Publish on NAME/joint_states
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

    //Publish on NAME/curr_vel
    curr_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("curr_vel", 1);

    state_ = s;
    state_->buttons[0] = 0;
    state_->buttons[1] = 0;
    state_->buttons_prev[0] = 0;
    state_->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state_->velocity = zeros;
    state_->inp_vel1 = zeros;  //3x1 history of velocity
    state_->inp_vel2 = zeros;  //3x1 history of velocity
    state_->inp_vel3 = zeros;  //3x1 history of velocity
    state_->out_vel1 = zeros;  //3x1 history of velocity
    state_->out_vel2 = zeros;  //3x1 history of velocity
    state_->out_vel3 = zeros;  //3x1 history of velocity
    state_->pos_hist1 = zeros; //3x1 history of position
    state_->pos_hist2 = zeros; //3x1 history of position
    state_->lock = false;
    state_->apply_force = true;
    state_->lock_pos = zeros;
    if (!units.compare("mm"))
      state_->units_ratio = 1.0;
    else if (!units.compare("cm"))
      state_->units_ratio = 10.0;
    else if (!units.compare("dm"))
      state_->units_ratio = 100.0;
    else if (!units.compare("m"))
      state_->units_ratio = 1000.0;
    else
    {
      state_->units_ratio = 1.0;
      ROS_WARN("Unknown units [%s] using [mm]", units.c_str());
      units = "mm";
    }
    ROS_INFO("PHaNTOM position given in [%s], ratio [%.1f]", units.c_str(), state_->units_ratio);
  }

  /*******************************************************************************
   ROS node callback.
   *******************************************************************************/
  void force_callback(const omni_msgs::OmniFeedbackConstPtr& omnifeed) {
    ////////////////////Some people might not like this extra damping, but it
    ////////////////////helps to stabilize the overall force feedback. It isn't
    ////////////////////like we are getting direct impedance matching from the
    ////////////////////omni anyway
    state_->force[0] = omnifeed->force.x - 0.001 * state_->velocity[0];
    state_->force[1] = omnifeed->force.y - 0.001 * state_->velocity[1];
    state_->force[2] = omnifeed->force.z - 0.001 * state_->velocity[2];

    state_->lock_pos[0] = omnifeed->position.x;
    state_->lock_pos[1] = omnifeed->position.y;
    state_->lock_pos[2] = omnifeed->position.z;
  }

  void publish_omni_state() {
    // Build the state msg
    omni_msgs::OmniState state_msg;
    // Locked
    state_msg.locked = state_->lock;
    state_msg.close_gripper = state_->apply_force;
    // Position
    state_msg.pose.position.x = state_->position[0];
    state_msg.pose.position.y = state_->position[1];
    state_msg.pose.position.z = state_->position[2];
    // Orientation
    state_msg.pose.orientation.x = state_->rot.v()[0];
    state_msg.pose.orientation.y = state_->rot.v()[1];
    state_msg.pose.orientation.z = state_->rot.v()[2];
    state_msg.pose.orientation.w = state_->rot.s();
    // Velocity
    state_msg.velocity.x = state_->velocity[0];
    state_msg.velocity.y = state_->velocity[1];
    state_msg.velocity.z = state_->velocity[2];
    // TODO: Append Current to the state msg
    state_msg.header.stamp = ros::Time::now();
    state_pub_.publish(state_msg);
    
    // Publish the JointState msg
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "waist";
    joint_state.position[0] = -state_->thetas[1];
    joint_state.name[1] = "shoulder";
    joint_state.position[1] = state_->thetas[2];
    joint_state.name[2] = "elbow";
    joint_state.position[2] = state_->thetas[3];
    joint_state.name[3] = "yaw";
    joint_state.position[3] = -state_->thetas[4] + M_PI;
    joint_state.name[4] = "pitch";
    joint_state.position[4] = -state_->thetas[5] - 3*M_PI/4;
    joint_state.name[5] = "roll";
    joint_state.position[5] = -state_->thetas[6] - M_PI;
    joint_pub_.publish(joint_state);
    
    // Build the pose msg
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = state_msg.header;
    pose_msg.header.frame_id = ref_frame;
    pose_msg.pose = state_msg.pose;
    pose_msg.pose.position.x /= state_->units_ratio;
    pose_msg.pose.position.y /= state_->units_ratio;
    pose_msg.pose.position.z /= state_->units_ratio;
    pose_pub_.publish(pose_msg);

    //Enter only if at least one button changed state
    if ((state_->buttons[0] != state_->buttons_prev[0]) || (state_->buttons[1] != state_->buttons_prev[1]))
    {
      if (state_->buttons[0] == 1)
        state_->apply_force = !(state_->apply_force);
      if (state_->buttons[1] == 1)
        state_->lock = !(state_->lock);
      omni_msgs::OmniButtonEvent button_event;
      button_event.grey_button = state_->buttons[0];
      button_event.white_button = state_->buttons[1];
      state_->buttons_prev[0] = state_->buttons[0];
      state_->buttons_prev[1] = state_->buttons[1];
      button_pub_.publish(button_event);
    }

    //Build the velocity msg
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header = state_msg.header;
    vel_msg.header.frame_id = ref_frame;
    vel_msg.twist.linear.x = state_->velocity[0] / state_->units_ratio;
    vel_msg.twist.linear.y = state_->velocity[1] / state_->units_ratio;
    vel_msg.twist.linear.z = state_->velocity[2] / state_->units_ratio;
    curr_vel_pub_.publish(vel_msg);
  }
};

HDCallbackCode omni_state_callback(void *pUserData)
{
  OmniState *omni_state = (OmniState *) pUserData;
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
  {
    ROS_DEBUG("Updating calibration...");
    hdUpdateCalibration(calibrationStyle);
  }
  hdBeginFrame(hdGetCurrentDevice());


  // Get transform and angles
  hduMatrix transform;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);

  hduMatrix rotation(transform);

  omni_state->position = hduVector3Dd(transform[3][0], transform[3][1], transform[3][2]);
  omni_state->rot = hduQuaternion(rotation);

/*
  hduVector3Dd velocities;
  hdGetDoublev(HD_CURRENT_VELOCITY, velocities);

  std::cout <<
    velocities[0] << ", " << velocities[1] << ", " << velocities[2] << "\r";
*/

  // Velocity estimation
  hduVector3Dd vel_buff(0, 0, 0);

  //2nd order backward dif, Assuming that the rate is 1kHz.
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
      + omni_state->pos_hist2) / 0.002;  //(units)/s, 
  omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
      + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
          - 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz

  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;
  
  

  // Set forces if locked
  if (omni_state->lock == true) {
    omni_state->force = 0.04 * (omni_state->lock_pos - omni_state->position) / omni_state->units_ratio
      - 0.001 * omni_state->velocity;
  }
  hduVector3Dd feedback;

  if(omni_state->apply_force)
  {
    feedback[0] = omni_state->force[0];
    feedback[1] = omni_state->force[1];
    feedback[2] = omni_state->force[2];
  }
  
  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  //Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
      omni_state->joints[2] - omni_state->joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];
  return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration()
{
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
  {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("HD_CALIBRATION_ENCODER_RESET..");
  }

  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
  {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("HD_CALIBRATION_INKWELL..");
  }

  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
  {
    calibrationStyle = HD_CALIBRATION_AUTO;
    ROS_INFO("HD_CALIBRATION_AUTO..");
  }

  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET)
  {
    do
    {
      hdUpdateCalibration(calibrationStyle);
      ROS_INFO("Calibrating.. (put stylus into the inkwell)");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    }while (hdCheckCalibration() != HD_CALIBRATION_OK);
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK)
  {
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) 
      ROS_WARN("Please place the device into the inkwell for calibration.");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
    {
      ROS_INFO("Calibration updated successfully");
      hdUpdateCalibration(calibrationStyle);
    }
    else
      ROS_FATAL("Unknown calibration status");
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("Calibration complete.");
}

void *ros_publish(void *ptr)
{
  PhantomROS *omni_ros = (PhantomROS *) ptr;
  int publish_rate;
  ros::param::param(std::string("~/publish_rate"), publish_rate, 1000);
  ROS_INFO("Publishing PHaNTOM state at [%d] Hz", publish_rate);
  ros::Rate loop_rate(publish_rate);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok())
  {
    omni_ros->publish_omni_state();
    loop_rate.sleep();
  }
  return NULL;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "omni_haptic_node");
  ros::NodeHandle nh;
  OmniState state;

  ////////////////////////////////////////////////////////////////
  // Init Phantom
  ////////////////////////////////////////////////////////////////
  HDErrorInfo error;
  HHD hHD;
  hHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    //hduPrintError(stderr, &error, "Failed to initialize haptic device");
    ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
    return -1;
  }

  ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    ROS_ERROR("Failed to start the scheduler"); //, &error);
    return -1;
  }
  HHD_Auto_Calibration();

  ////////////////////////////////////////////////////////////////
  // Init ROS
  ////////////////////////////////////////////////////////////////
  PhantomROS omni_ros;

  omni_ros.init(&state);
  hdScheduleAsynchronous(omni_state_callback, &state,
      HD_MAX_SCHEDULER_PRIORITY);

  ////////////////////////////////////////////////////////////////
  // Loop and publish
  ////////////////////////////////////////////////////////////////
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
  pthread_join(publish_thread, NULL);

  ROS_INFO("Ending Session....");
  hdStopScheduler();
  hdDisableDevice(hHD);

  return 0;
}
