#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#define G  9.80665

struct IMU_DATA
{
  double Ax; // Accel x axis
  double Ay; // Accel y axis
  double Az; // Accel z axis
  double Gx; // Gyro x axis
  double Gy; // Gyro y axis
  double Gz; // Gyro z axis
  double Roll; // Roll
  double Pitch; // Pitch
  double Yaw; // Yaw
  double Temp; // Temperature
};

inline double deg2rad(double deg)
{
    return deg * M_PI / 180.;
}

std::string stringToHex(const std::string& input)
{
  static const char* const lut = "0123456789ABCDEF";
  std::size_t len = input.length();

  std::string output;
  output.reserve(2 * len);
  for (std::size_t i = 0; i < len; ++i)
  {
    const unsigned char c = input[i];
    output.push_back(lut[c >> 4]);
    output.push_back(lut[c & 15]);
  }
  return output;
}

std::string char_to_hex(char c){
   static const char* const lut = "0123456789ABCDEF";
   return ""+lut[c >> 4] + lut[c & 15]; 
}

int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string imu_frame_id;
  std::string imu_topic;

  ros::init(argc, argv, "ray_imu_node");
  int cfg_baud_rate = 115200;
  static const int Sensor_Scale = 65536;
  static const int Angle_Scale = 360;
  static const int Accel_Scale = 20;
  static const int Rate_Scale = 1260;
  static const int Temp_Scale = 200;
  static const int MinLength = 20;
  
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
  private_nh.param<std::string>("imu_frame_id", imu_frame_id, "imu_base");
  private_nh.param<int>("imu_baudrate", cfg_baud_rate, 115200);
  private_nh.param<std::string>("imu_topic", imu_topic, "imu");

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 50);
  ros::Publisher imu_angle_pub = nh.advertise<std_msgs::Float32>("imu_angle", 50);

  ros::Rate r(200); // 1000 hz

  uint8_t size=23;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Duration dur_time;
  IMU_DATA imuData;
  double sensors[10];

  try
  {
    ser.setPort(port);
    ser.setBaudrate(cfg_baud_rate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    ros::shutdown();
  }

  if(ser.isOpen())
  {
    ROS_INFO_STREAM("Serial Port initialized ok");
  }
  else
  {
    ROS_INFO_STREAM("Serial Port initialized failed");
    ros::shutdown();
  }

  ROS_INFO_STREAM("imu start work...");

  while(ser.available() > 0) {
    std::string t = ser.read(1);
  }

  while(ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        if(ser.available())
        {
          char cmd[46];
          unsigned char cmd1[46];

          std::string input_0 = ser.read(size);
          std::string input_hex = stringToHex(input_0);
          //ROS_ERROR_STREAM("input_hex: " << input_hex);
          
          std::string input;
          std::string flag;
          std::string::size_type position;
          position = input_hex.find("7F80");
          if (position != input_hex.npos || (input_hex.find("7F")==44)) 
          {
              //ROS_ERROR_STREAM("position: " << position);
              if (input_hex.find("7F")==44) 
              {
                  position = 44;
              }
              //ROS_ERROR_STREAM("input_str: " << position);
              if(position!=0)
              {
                  std::string input_1 = ser.read((position/2)); 
                  input=input_0+input_1;
              }
              else
              {
                  input=input_0;
              }
       
              char *chr = &input[0u+position/2];

              sensors[0] = (short)((chr[2] & 0xFF) | ((chr[3] << 8) & 0XFF00));
              sensors[1] = (short)((chr[4] & 0xFF) | ((chr[5] << 8) & 0XFF00));
              sensors[2] = (short)((chr[6] & 0xFF) | ((chr[7] << 8) & 0XFF00));

              sensors[3] = (short)((chr[8] & 0xFF) | ((chr[9] << 8) & 0XFF00));
              sensors[4] = (short)((chr[10] & 0xFF) | ((chr[11] << 8) & 0XFF00));
              sensors[5] = (short)((chr[12] & 0xFF) | ((chr[13] << 8) & 0XFF00));

              sensors[6] = (short)((chr[14] & 0xFF) | ((chr[15] << 8) & 0XFF00));
              sensors[7] = (short)((chr[16] & 0xFF) | ((chr[17] << 8) & 0XFF00));
              sensors[8] = (short)((chr[18] & 0xFF) | ((chr[19] << 8) & 0XFF00));
              sensors[9] = (short)((chr[20] & 0xFF) | ((chr[21] << 8) & 0XFF00));
              
              imuData.Ax = sensors[0] * Accel_Scale / Sensor_Scale;
              imuData.Ay = sensors[1] * Accel_Scale / Sensor_Scale;
              imuData.Az = sensors[2] * Accel_Scale / Sensor_Scale;

              imuData.Gx = sensors[3] * Rate_Scale / Sensor_Scale;
              imuData.Gy = sensors[4] * Rate_Scale / Sensor_Scale;
              imuData.Gz = sensors[5] * Rate_Scale / Sensor_Scale;

              imuData.Roll = sensors[6] * Angle_Scale / Sensor_Scale;
              imuData.Pitch = sensors[7] * Angle_Scale / Sensor_Scale;
              imuData.Yaw = sensors[8] * Angle_Scale / Sensor_Scale;
              imuData.Temp = sensors[9] * Temp_Scale / Sensor_Scale;

              current_time = ros::Time::now();
              dur_time = current_time - last_time;

              //ROS_ERROR_STREAM("acc_x: " << imuData.Ax<<"   acc_y: " << imuData.Ay<<"  acc_z: " << imuData.Az);
              //ROS_ERROR_STREAM("gray_x: " <<imuData.Gx<<"   gray_y: " <<imuData.Gy<<"  gray_z: " <<imuData.Gz);
              //ROS_ERROR_STREAM("roll: " <<imuData.Roll<<"   pitch: " <<imuData.Pitch<<"  yaw: " <<imuData.Yaw <<"  temp: " <<imuData.Temp);

              std_msgs::Float32 imu_angle_mgs;
              imu_angle_mgs.data = imuData.Yaw;
              imu_angle_pub.publish(imu_angle_mgs);

              ros::Time measurement_time = ros::Time::now();
              sensor_msgs::Imu imu;
              imu.header.stamp = measurement_time;
              imu.header.frame_id = imu_frame_id;

              double roll_data = imuData.Roll*M_PI/(180);
              double pitch_data = imuData.Pitch*M_PI/(180);
              double yaw_data = imuData.Yaw*M_PI/(180);

              geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll_data, -pitch_data, -yaw_data);
              imu.orientation_covariance[0] = 0;
              imu.orientation_covariance[1] = 0;
              imu.orientation_covariance[2] = 0;
              imu.orientation_covariance[3] = 0;
              imu.orientation_covariance[4] = 0;
              imu.orientation_covariance[5] = 0;
              imu.orientation_covariance[6] = 0;
              imu.orientation_covariance[7] = 0;
              imu.orientation_covariance[8] = 0;
              imu.orientation = odom_quat;

              imu.angular_velocity.x = (double)(imuData.Gx*M_PI/(180));
              imu.angular_velocity.y = (double)(-1*imuData.Gy*M_PI/(180));
              imu.angular_velocity.z = (double)(-1*imuData.Gz*M_PI/(180));

              imu.linear_acceleration.x = (double)(((imuData.Ax)*G));
              imu.linear_acceleration.y = (double)(((-1*imuData.Ay)*G));
              imu.linear_acceleration.z = (double)((-1*imuData.Az)*G);

              imu_pub.publish(imu);          
          }
      }
      else
      {
          //ROS_WARN_STREAM("NOT found " );
      } 

      }
      else
      {
        // try and open the serial port
        try
        {
          ser.setPort(port);
          ser.setBaudrate(cfg_baud_rate);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }
      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
}
