#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define PepperFSMController_DLLIMPORT __declspec(dllimport)
#  define PepperFSMController_DLLEXPORT __declspec(dllexport)
#  define PepperFSMController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define PepperFSMController_DLLIMPORT __attribute__((visibility("default")))
#    define PepperFSMController_DLLEXPORT __attribute__((visibility("default")))
#    define PepperFSMController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define PepperFSMController_DLLIMPORT
#    define PepperFSMController_DLLEXPORT
#    define PepperFSMController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef PepperFSMController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define PepperFSMController_DLLAPI
#  define PepperFSMController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef PepperFSMController_EXPORTS
#    define PepperFSMController_DLLAPI PepperFSMController_DLLEXPORT
#  else
#    define PepperFSMController_DLLAPI PepperFSMController_DLLIMPORT
#  endif // PepperFSMController_EXPORTS
#  define PepperFSMController_LOCAL PepperFSMController_DLLLOCAL
#endif // PepperFSMController_STATIC
