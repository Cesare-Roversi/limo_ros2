#ifndef LIMO_PLANNER_CPP__VISIBILITY_CONTROL_H_
#define LIMO_PLANNER_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LIMO_PLANNER_CPP_EXPORT __attribute__ ((dllexport))
    #define LIMO_PLANNER_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define LIMO_PLANNER_CPP_EXPORT __declspec(dllexport)
    #define LIMO_PLANNER_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef LIMO_PLANNER_CPP_BUILDING_DLL
    #define LIMO_PLANNER_CPP_PUBLIC LIMO_PLANNER_CPP_EXPORT
  #else
    #define LIMO_PLANNER_CPP_PUBLIC LIMO_PLANNER_CPP_IMPORT
  #endif
  #define LIMO_PLANNER_CPP_PUBLIC_TYPE LIMO_PLANNER_CPP_PUBLIC
  #define LIMO_PLANNER_CPP_LOCAL
#else
  #define LIMO_PLANNER_CPP_EXPORT __attribute__ ((visibility("default")))
  #define LIMO_PLANNER_CPP_IMPORT
  #if __GNUC__ >= 4
    #define LIMO_PLANNER_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define LIMO_PLANNER_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LIMO_PLANNER_CPP_PUBLIC
    #define LIMO_PLANNER_CPP_LOCAL
  #endif
  #define LIMO_PLANNER_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // LIMO_PLANNER_CPP__VISIBILITY_CONTROL_H_