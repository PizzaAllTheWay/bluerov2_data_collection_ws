#include <unistd.h>
#include <fcntl.h>
#include <cstring>
// #include <iostream>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <wiringPiI2C.h>
#include <cmath>
#include <bluerov_actuator_driver/PCA9685.h>
#include <bluerov_actuator_driver/Constants.h>

PCA9685::PCA9685(const std::string & device, int address)
{
  if ((i2c_fd = open(device.c_str(), O_RDWR)) < 0) {
    // std::cout << "open: " << std::strerror(errno) << "\n";
    exit(1);
  }
  int addr = address;
  if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
    // std::cout << "ioctl: " << std::strerror(errno) << "\n";
    exit(1);
  }
  set_all_pwm(0, 0);
  auto ret = wiringPiI2CWriteReg8(i2c_fd, MODE2, OUTDRV);
  check_ret(ret, "set mode2");
  ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, ALLCALL);
  check_ret(ret, "set mode1");
  usleep(5000);
  auto mode1 = wiringPiI2CReadReg8(i2c_fd, MODE1);
  check_ret(mode1, "read mode1");
  mode1 = mode1 & ~SLEEP;
  ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, mode1);
  check_ret(ret, "write mode1");
  usleep(5000);
}

void PCA9685::set_pwm_freq(double freq_hz)
{
  frequency = freq_hz;

  // auto prescaleval = 25000000.0; //    # 25MHz
  // prescaleval /= 4096.0; //       # 12-bit
  // prescaleval /= freq_hz;
  // prescaleval -= 1.0;

  // auto prescale = static_cast<int>(std::round(prescaleval));
  float prescaleval = ((FREQUENCY_OSCILLATOR / (frequency * 4096.0)) + 0.5) - 1; // more accurate according to datasheet
  // float prescaleval = ((FREQUENCY_OSCILLATOR / (frequency * 4096.0)) + 0.5); // works better with T200 thruster
  // float prescaleeval = 128;
  if (prescaleval < PCA9685_PRESCALE_MIN) {
    prescaleval = PCA9685_PRESCALE_MIN;
  }
  if (prescaleval > PCA9685_PRESCALE_MAX) {
    prescaleval = PCA9685_PRESCALE_MAX;
  }
  uint8_t prescale = (uint8_t)prescaleval;

  prescale_ = prescale;

  auto oldmode = wiringPiI2CReadReg8(i2c_fd, MODE1);
  check_ret(oldmode);

  auto newmode = (oldmode & ~RESTART) | SLEEP;
  auto ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, newmode);
  check_ret(ret);
  ret = wiringPiI2CWriteReg8(i2c_fd, PRESCALE, prescale_);
  check_ret(ret);
  ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, oldmode);
  check_ret(ret);
  usleep(5000);
  ret = wiringPiI2CWriteReg8(i2c_fd, MODE1, oldmode | RESTART | MODE1_AI);
  check_ret(ret);
}

void PCA9685::set_pwm(int channel, uint16_t on, uint16_t off)
{
  wiringPiI2CWriteReg8(i2c_fd, LED0_ON_L + 4 * channel, on & 0xFF);
  wiringPiI2CWriteReg8(i2c_fd, LED0_ON_H + 4 * channel, on >> 8);
  wiringPiI2CWriteReg8(i2c_fd, LED0_OFF_L + 4 * channel, off & 0xFF);
  wiringPiI2CWriteReg8(i2c_fd, LED0_OFF_H + 4 * channel, off >> 8);
}

void PCA9685::set_all_pwm(uint16_t on, uint16_t off)
{
  wiringPiI2CWriteReg8(i2c_fd, ALL_LED_ON_L, on & 0xFF);
  wiringPiI2CWriteReg8(i2c_fd, ALL_LED_ON_H, on >> 8);
  wiringPiI2CWriteReg8(i2c_fd, ALL_LED_OFF_L, off & 0xFF);
  wiringPiI2CWriteReg8(i2c_fd, ALL_LED_OFF_H, off >> 8);
}

void PCA9685::set_pwm_ms(int channel, double ms)
{
  auto period_ms = 1000.0 / frequency;
  auto bits_per_ms = 4096 / period_ms;
  auto bits = ms * bits_per_ms;
  set_pwm(channel, 0, bits);
}

void PCA9685::set_pwm_us(int channel, uint32_t us)
{
  double pulse = us;
  double pulselength;
  pulselength = 1000000; // 1,000,000 us per second

  uint16_t prescale = prescale_;
  prescale += 1;
  pulselength *= prescale;
  pulselength /= FREQUENCY_OSCILLATOR;

  pulse /= pulselength;

  set_pwm(channel, 0, pulse);
}

void PCA9685::check_ret(int ret, std::string msg)
{
  if (ret < 0) {
    // std::cerr << "ERROR" << std::endl;
    // std::cerr << std::strerror(errno) << std::endl;
    // std::cerr << msg << std::endl;
    exit(1);
  }
}
