#ifndef __ETL_PROFILE_H__
#define __ETL_PROFILE_H__

//#define ETL_THROW_EXCEPTIONS
#define ETL_VERBOSE_ERRORS
#define ETL_CHECK_PUSH_POP

#ifdef _MSC_VER
  #include "profiles/msvc.h"
#else
  #include "profiles/cpp03.h"
#endif
#endif