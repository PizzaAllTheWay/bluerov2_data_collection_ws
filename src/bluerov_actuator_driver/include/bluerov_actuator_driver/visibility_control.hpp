#ifndef BLUEROV_ACTUATOR_DRIVER__VISIBILITY_CONTROL_HPP_
#define BLUEROV_ACTUATOR_DRIVER__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BLUEROV_ACTUATOR_DRIVER_EXPORT __attribute__((dllexport))
#define BLUEROV_ACTUATOR_DRIVER_IMPORT __attribute__((dllimport))
#else
#define BLUEROV_ACTUATOR_DRIVER_EXPORT __declspec(dllexport)
#define BLUEROV_ACTUATOR_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef BLUEROV_ACTUATOR_DRIVER_BUILDING_DLL
#define BLUEROV_ACTUATOR_DRIVER_PUBLIC BLUEROV_ACTUATOR_DRIVER_EXPORT
#else
#define BLUEROV_ACTUATOR_DRIVER_PUBLIC BLUEROV_ACTUATOR_DRIVER_IMPORT
#endif
#define BLUEROV_ACTUATOR_DRIVER_PUBLIC_TYPE BLUEROV_ACTUATOR_DRIVER_PUBLIC
#define BLUEROV_ACTUATOR_DRIVER_LOCAL
#else
#define BLUEROV_ACTUATOR_DRIVER_EXPORT __attribute__((visibility("default")))
#define BLUEROV_ACTUATOR_DRIVER_IMPORT
#if __GNUC__ >= 4
#define BLUEROV_ACTUATOR_DRIVER_PUBLIC __attribute__((visibility("default")))
#define BLUEROV_ACTUATOR_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define BLUEROV_ACTUATOR_DRIVER_PUBLIC
#define BLUEROV_ACTUATOR_DRIVER_LOCAL
#endif
#define BLUEROV_ACTUATOR_DRIVER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif // BLUEROV_ACTUATOR_DRIVER__VISIBILITY_CONTROL_HPP_