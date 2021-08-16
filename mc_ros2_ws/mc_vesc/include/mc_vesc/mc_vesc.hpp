// 16.Aug.2021
// Jungill Kang

#ifndef MC_VESC__MC_VESC_HPP_
#define MC_VESC__MC_VESC_HPP_

// For smart pointers
#include <memory>

// ros dependencies
#include <rclcpp/rclcpp.hpp>

// For UART communication
#include <errno.h> /* ERROR Number Definitions          */
#include <fcntl.h> /* File Control Definitions          */
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions */
#include <cmath>

// basic lib
#include <chrono>
#include <string>

// msg library
#include <geometry_msgs/msg/twist.hpp>

// for the duration of timer callback
using namespace std::chrono_literals;

/// Indicates the motor number
enum MotorID
{
  /// motor 1
  MOTOR1 = 0,
  /// motor 2
  MOTOR2 = 1,
};

namespace McVesc{

class McVesc : public rclcpp::Node
{
public:
  McVesc();
  ~McVesc();

private:
  // member function def
  // member function def
  // member function def
  
  // Serial comm
  int open_serial(char *dev_name, int baud, int vtime, int vmin);
  void close_serial(int fd);

  void uart_pub_callback() const;
  void cmd_vel_sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // motor control
  void set_motor_release(int fd, unsigned char ID) const;
  void motor_flush(int fd) const;
  void set_motor_brake(int fd, unsigned char ID) const;
  void set_motor_vel(int fd, unsigned char ID, float velocity) const;

  // member variable def
  // member variable def
  // member variable def

  // UART comm handle
  int fd_;

  // Ros publisher & subscriber
  rclcpp::TimerBase::SharedPtr uart_timer_;
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

  // rpm value for the motor
  float rpm_;

}; // class def end

} // namespace end

#endif
