#ifndef CSE_ACTUATOR_DRIVER__PIXHAWK_H_
#define CSE_ACTUATOR_DRIVER__PIXHAWK_H_


#include <string>
#include <tuple>
#include <cstdint>
// namespace cse_actuator_driver
// {

class Pixhawk
{
public:
  explicit Pixhawk(const char * device = "/dev/ttyS4");
  ~Pixhawk();

  // Sets pwm
  void set_pwm_us(uint32_t* pwm, uint32_t size);
  void set_io_pwm_us(uint32_t* pwm, uint32_t size);

  std::tuple<float, float> get_battery_status();

private:

  bool sync_package();

  int uart_e_;
  int max_bytes_to_read_{20};
};

#endif