// 16.Aug.2021
// Jungill Kang

#include <stdexcept>
#include <mc_vesc/mc_vesc.hpp>

// for the binding function
using std::placeholders::_1;

namespace McVesc{

McVesc::McVesc()
: Node("vesc_velocity_controller")
{
  // create pub & sub
  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&McVesc::cmd_vel_sub_callback, this, _1));

  // initiallize the uart comm port
  // RS485 : "/dev/ttyUSBO" 
  try
  {
    fd_ = open_serial("/dev/ttyUSB0", 115200, 0, 0);

    RCLCPP_INFO(this->get_logger(), "open");
    // open error
    if (fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Can't open serial port");
      throw std::invalid_argument("Can't open serial port");
      rclcpp::shutdown();
    }
  }
  catch (std::invalid_argument & e)
  {
    std::cerr << e.what() << std::endl;
    close_serial(fd_);
  }


  if(fd_ > 0){
  set_motor_brake(fd_, 1);

  // 100Hz velocity command
  uart_timer_ = this->create_wall_timer(
    10ms, std::bind(&McVesc::uart_pub_callback, this));
  }
}

McVesc::~McVesc()
{
  close_serial(fd_);
}

int McVesc::open_serial(char *dev_name, int baud, int vtime, int vmin)
{
  int fd;
  struct termios newtio;
  // open serial port
  // fd = open(dev_name, O_RDWR | O_NOCTTY);
  fd = open(dev_name, O_WRONLY | O_NOCTTY);

  RCLCPP_INFO(this->get_logger(), "%d", fd);

  if (fd < 0)
  {
      // error
      printf("Device OPEN FAIL %s\n", dev_name);
      return -1;
  }
  // serial comm setting
  memset(&newtio, 0, sizeof(newtio));
  newtio.c_iflag = IGNPAR; // non-parity
  // newtio.c_iflag = 0;
  newtio.c_oflag = 0;
  newtio.c_cflag = CS8 | CLOCAL | CREAD; // NO-rts/cts
  switch (baud)
  {
  case 115200:
      newtio.c_cflag |= B115200;
      break;
  case 57600:
      newtio.c_cflag |= B57600;
      break;
  case 38400:
      newtio.c_cflag |= B38400;
      break;
  case 19200:
      newtio.c_cflag |= B19200;
      break;
  case 9600:
      newtio.c_cflag |= B9600;
      break;
  case 4800:
      newtio.c_cflag |= B4800;
      break;
  case 2400:
      newtio.c_cflag |= B2400;
      break;
  default:
      newtio.c_cflag |= B115200;
      break;
  }
  // set input mode (non-canonical, no echo,.....)
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = vtime; // timeout 0.1s
  newtio.c_cc[VMIN] = vmin;   // wait untill n characters
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  return fd;
}

void McVesc::close_serial(int fd) { close(fd); }

void McVesc::uart_pub_callback() const
{
  set_motor_vel(fd_, MotorID::MOTOR1, rpm_);

  RCLCPP_INFO(this->get_logger(), "PUB rpm : %f", rpm_);
}

void McVesc::cmd_vel_sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  rpm_ = (msg->linear.x) * 100;

  // RCLCPP_INFO(this->get_logger(), "GET : %f", msg->linear.x);
}

void McVesc::set_motor_release(int fd, unsigned char ID) const
{
  unsigned char outbuff[8];
  outbuff[0] = 0x21;
  outbuff[1] = 0xfe;
  outbuff[2] = ID;
  outbuff[3] = 203;
  outbuff[4] = 0xfe;
  outbuff[5] = 0xfe;
  outbuff[6] = 0xfe;
  outbuff[7] = 0xfe;
  write(fd, outbuff, 8);
}

void McVesc::motor_flush(int fd) const
{
  unsigned char outbuff[8];
  outbuff[0] = 0xfc;
  outbuff[1] = 0xfc;
  outbuff[2] = 0xfc;
  outbuff[3] = 0xfc;
  outbuff[4] = 0xfc;
  outbuff[5] = 0xfc;
  outbuff[6] = 0xfc;
  outbuff[7] = 0xfc;
  write(fd, outbuff, 8);
}

void McVesc::set_motor_brake(int fd, unsigned char ID) const
{
  unsigned char outbuff[8];
  outbuff[0] = 0x21;
  outbuff[1] = 0xfe;
  outbuff[2] = ID;
  outbuff[3] = 202;
  outbuff[4] = 0xfe;
  outbuff[5] = 0xfe;
  outbuff[6] = 0xfe;
  outbuff[7] = 0xfe;
  write(fd, outbuff, 8);
}

void McVesc::set_motor_vel(int fd, unsigned char ID, float velocity) const
{
  unsigned char outbuff[8];

  outbuff[0] = 0x21;
  outbuff[1] = 0xfe;
  outbuff[2] = ID;
  outbuff[3] = 0xfe;

  static_assert(sizeof(float) == 4);

  memcpy(&outbuff[4], &velocity, sizeof(float));

  RCLCPP_INFO(this->get_logger(), "MEM : %u %u %u %u", outbuff[4], outbuff[5], outbuff[6], outbuff[7]);

  write(fd, outbuff, 8);
}

} // namespace end

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<McVesc::McVesc>());
  rclcpp::shutdown();
  return 0;
}