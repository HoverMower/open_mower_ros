//
// Created by Patrick Weber on 10.06.22.
// Copyright (c) 2022 Patrick Weber. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geographic_msgs/GeoPose.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <mower_msgs/Status.h>
#include "rosmower_msgs/Battery.h"
#include "rosmower_msgs/MowMotor.h"
#include "rosmower_msgs/setMowMotor.h"
//#include "<mower_msgs/HighLevelControlSrv.h>"
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "robot_localization/SetDatum.h"

ros::Publisher status_pub;
ros::Publisher estop_pub;
ros::Subscriber battery_sub;
ros::Subscriber mow_motor_sub;
ros::Subscriber gps_fix_sub;
ros::Subscriber imu_sub;
ros::ServiceClient srv_mow;
ros::ServiceClient srv_gps_datum;

// True, if ROS thinks there sould be an emergency
bool emergency_high_level = false;
// True, if the LL board thinks there should be an emergency
bool emergency_low_level = false;

// True, if the LL emergency should be cleared in the next request
bool ll_clear_emergency = false;

// true, if gps datum has been provided to robot_localization
bool gps_datum_provided = false;

// Current speeds
float speed_mow = 0;

ros::Time last_cmd_vel(0.0);

ros::ServiceClient highLevelClient;

rosmower_msgs::Battery last_battery_msg;
rosmower_msgs::MowMotor last_mow_msg;
sensor_msgs::Imu last_imu;

bool is_emergency()
{
    return emergency_high_level || emergency_low_level;
}

void batteryCallback(const rosmower_msgs::Battery::ConstPtr &msg)
{
    last_battery_msg = *msg;
}

void mowMotorCallback(const rosmower_msgs::MowMotor::ConstPtr &msg)
{
    last_mow_msg = *msg;
    emergency_low_level = msg->alarm;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    last_imu = *msg;
}

void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (gps_datum_provided == false)
    {
        if (msg->status.status > 0)
        {
            geographic_msgs::GeoPose gps_datum;
            
            robot_localization::SetDatum srv;

            gps_datum.position.latitude = msg->latitude;
            gps_datum.position.longitude = msg->longitude;
            gps_datum.position.altitude = msg->altitude;

            gps_datum.orientation = last_imu.orientation;

            srv.request.geo_pose = gps_datum;
            srv_gps_datum.call(srv);

            ROS_INFO_STREAM("Setting gps start position");
            gps_datum_provided = true;
        }
    }
}

void convertStatus(rosmower_msgs::MowMotor &mow_status, mower_msgs::ESCStatus &ros_esc_status)
{
    if (mow_status.alarm)
    {
        // ESC has a fault
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
    }
    else if (abs(mow_status.speed) > 500)
    {
        // ESC is running
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_RUNNING;
    }
    else
    {
        // ESC is OK but standing still
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    }
    ros_esc_status.tacho = mow_status.speed;
    ros_esc_status.current = mow_status.current;
}

void publishStatus()
{
    mower_msgs::Status status_msg;
    status_msg.stamp = ros::Time::now();

    // if (last_ll_status.status_bitmask & 1) {
    // LL OK, fill the message
    //     status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
    // } else {
    // LL initializing
    //     status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_INITIALIZING;
    // }

    // status_msg.raspberry_pi_power = (last_ll_status.status_bitmask & 0b00000010) != 0;
    // status_msg.gps_power = (last_ll_status.status_bitmask & 0b00000100) != 0;
    // status_msg.esc_power = (last_ll_status.status_bitmask & 0b00001000) != 0;
    // status_msg.rain_detected = (last_ll_status.status_bitmask & 0b00010000) != 0;
    // status_msg.sound_module_available = (last_ll_status.status_bitmask & 0b00100000) != 0;
    // status_msg.sound_module_busy = (last_ll_status.status_bitmask & 0b01000000) != 0;
    // status_msg.ui_board_available = (last_ll_status.status_bitmask & 0b10000000) != 0;

    // for (uint8_t i = 0; i < 5; i++) {
    //    status_msg.ultrasonic_ranges[i] = last_ll_status.uss_ranges_m[i];
    //}

    // overwrite emergency with the LL value.
    // emergency_low_level = last_ll_status.emergency_bitmask > 0;
    // if (!emergency_low_level) {
    // it obviously worked, reset the request
    //    ll_clear_emergency = false;
    //} else {
    //    ROS_ERROR_STREAM_THROTTLE(1, "Low Level Emergency. Bitmask was: " << (int)last_ll_status.emergency_bitmask);
    //}

    // True, if high or low level emergency condition is present
    status_msg.emergency = is_emergency();

    status_msg.v_battery = last_battery_msg.battery_voltage;     // last_ll_status.v_system;
    status_msg.v_charge = last_battery_msg.charge_voltage;       // last_ll_status.v_charge;
    status_msg.charge_current = last_battery_msg.charge_current; // last_ll_status.charging_current;

    // vesc_driver::VescStatusStruct mow_status;
    // mow_vesc_interface->get_status(&mow_status);

    convertStatus(last_mow_msg, status_msg.mow_esc_status);

    status_pub.publish(status_msg);
}

void publishActuatorsTimerTask(const ros::TimerEvent &timer_event)
{
    publishStatus();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res)
{
    rosmower_msgs::setMowMotor srv;
    if (req.mow_enabled && !is_emergency())
    {
        speed_mow = 1;
        srv.request.Speed = 1500;
    }
    else
    {
        speed_mow = 0;
        srv.request.Speed = 0;
    }
    ROS_INFO_STREAM("Setting mow enabled to " << speed_mow);
    srv_mow.call(srv);
    return true;
}

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res)
{
    if (req.emergency)
    {
        ROS_ERROR_STREAM("Setting emergency!!");
        ll_clear_emergency = false;
    }
    else
    {
        ll_clear_emergency = true;
    }
    // Set the high level emergency instantly. Low level value will be set on next update.
    emergency_high_level = req.emergency;

    std_msgs::Bool msg_estop;
    msg_estop.data = is_emergency();
    estop_pub.publish(msg_estop);
    ros::spinOnce();

    return true;
}

// void handleLowLevelUIEvent(struct ui_command *ui_command) {
//     ROS_INFO_STREAM("Got UI button with code:" << ui_command->cmd1);

//     mower_msgs::HighLevelControlSrv srv;

//     switch(ui_command->cmd1) {
//         case 2:
//             // Home
//             srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME;
//             break;
//         case 3:
//             // Play
//             srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_START;
//             break;
//         case 4:
//             // S1
//             srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S1;
//             break;
//         case 5:
//             // S2
//             srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S2;
//             break;
//         default:
//             // Return, don't call the service.
//             return;
//     }

//     if(!highLevelClient.call(srv)) {
//         ROS_ERROR_STREAM("Error calling high level control service");
//     }

// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HoverMower_wrapper");

    ros::NodeHandle nh;
    ros::NodeHandle paramNh("~");

    // highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>(
    //         "mower_service/high_level_control");

    speed_mow = 0;

    status_pub = nh.advertise<mower_msgs::Status>("mower/status", 1);
    estop_pub = nh.advertise<std_msgs::Bool>("/e_stop", 3);
    battery_sub = nh.subscribe("hovermower/sensors/Battery", 1000, batteryCallback);
    mow_motor_sub = nh.subscribe("hovermower/sensors/MowMotor", 1000, mowMotorCallback);
    gps_fix_sub = nh.subscribe("/ublox/fix", 1000, gpsFixCallback);
    imu_sub = nh.subscribe("bno08x/raw", 1000, imuCallback);

    ros::ServiceServer mow_service = nh.advertiseService("mower_service/mow_enabled", setMowEnabled);
    ros::ServiceServer emergency_service = nh.advertiseService("mower_service/emergency", setEmergencyStop);
    srv_mow = nh.serviceClient<rosmower_msgs::setMowMotor>("hovermower/setMowMotorSpeed");
    srv_gps_datum = nh.serviceClient<robot_localization::SetDatum>("datum");

    ros::Timer publish_timer = nh.createTimer(ros::Duration(0.02), publishActuatorsTimerTask);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok())
    {
    }

    spinner.stop();
    return 0;
}
