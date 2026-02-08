#include <memory>
#include <string>
#include <stdio.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp"
// #include "pwm_interface/msg/pwm.hpp"

#include <bluerov_actuator_driver/pixhawk.hpp>

using std::placeholders::_1;

Pixhawk::Pixhawk(const char * device)
{
  uart_e_ = open(device, O_RDWR);

  if (uart_e_ < 0) {
    exit(1);
  }

  // Create a new termios struct
  struct termios tty;

  // Read in existing settings and handle any error
  if (tcgetattr(uart_e_, &tty) != 0) {
    exit(1);
  }

  // Control modes
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
  tty.c_cflag &= ~CSIZE; // Clear all the size bits
  tty.c_cflag |= CS8; // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  // Local modes
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  // Input modes
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  // Output modes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  // Vmin and Vtime
  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 128000 
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(uart_e_, TCSANOW, &tty) != 0) {
    exit(1);
  }

  // Enable pwm pins on pixhawk
  char cmd = 'e';
  write(uart_e_, &cmd, 1);
}

Pixhawk::~Pixhawk() {
  close(uart_e_);
}

void Pixhawk::set_pwm_us(uint32_t* pwm, uint32_t size)
{
  // Send
  char cmd = 's';
  write(uart_e_, &cmd, 1);
  write(uart_e_,(char*) pwm, size);
}

void Pixhawk::set_io_pwm_us(uint32_t* pwm, uint32_t size)
{
  // Send IO pwm (4 channels atm)
  char cmd = 'q';
  write(uart_e_, &cmd, 1);
  write(uart_e_,(char*) pwm, size);
}

std::tuple<float, float> Pixhawk::get_battery_status() 
{
  char cmd = 'r';
  write(uart_e_, &cmd, 1);

  // if (sync_package()) 
  {
    float buffer[2];
    read(uart_e_, (char*) buffer, 8);
    return {buffer[0], buffer[1]};
  }

  return {-1.0, -1.0};
}

bool Pixhawk::sync_package() 
{
  char buffer[2];
  int bytes_left_to_read = max_bytes_to_read_;

  read(uart_e_, buffer, 2);
  
  while (buffer[0] != 's' and buffer[1] != 'y')
  {
    buffer[0] = buffer[1];

    char* buf;
    read(uart_e_, buf, 1);
    buffer[1] = *buf;

    bytes_left_to_read--;
    if (bytes_left_to_read == 0) {
      return false;
    }
  }

  return true;

}
