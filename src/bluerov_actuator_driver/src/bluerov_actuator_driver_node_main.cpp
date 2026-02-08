#include <memory>
#include <utility>
#include <string>
#include <thread>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "bluerov_actuator_driver/bluerov_actuator_driver_node.hpp"
// #include "bluerov_actuator_driver_node.hpp"
#include "pendulum_utils/process_settings.hpp"
#include "pendulum_utils/lifecycle_autostart.hpp"

// #include "tlsf_cpp/tlsf.hpp"

int main(int argc, char * argv[])
{
  pendulum::utils::ProcessSettings settings;
  if (!settings.init(argc, argv)) {
    return EXIT_FAILURE;
  }

  int32_t ret = 0;
  try {
    // configure process real-time settings
    if (settings.configure_child_threads) {
      // process child threads created by ROS nodes will inherit the settings
      settings.configure_process();
    }
    rclcpp::init(argc, argv);

    // Create a static executor
    rclcpp::executors::StaticSingleThreadedExecutor exec;

    // Create bluerov actuator node
    const auto bluerov_actuator_driver_ptr =
      std::make_shared<bluerov_actuator_driver::BlueROVActuatorDriverNode>("bluerov_actuator_driver");

    exec.add_node(bluerov_actuator_driver_ptr->get_node_base_interface());

    // configure process real-time settings
    if (!settings.configure_child_threads) {
      // process child threads created by ROS nodes will NOT inherit the settings
      settings.configure_process();
    }

    if (settings.auto_start_nodes) {
      pendulum::utils::autostart(*bluerov_actuator_driver_ptr);
    }

    exec.spin();
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("bluerov_actuator_driver"), e.what());
    ret = 2;
  } catch (...) {
    RCLCPP_INFO(
      rclcpp::get_logger("bluerov_actuator_driver"), "Unknown exception caught. "
      "Exiting...");
    ret = -1;
  }
  return ret;
}
