#ifndef NATIVE_TEST_HELPER_H
#define NATIVE_TEST_HELPER_H

#include <Arduino.h>
#include <BlinkBuzz/BlinkBuzz.h>
#include "NativeFileLog.h"
#include <string>
#if defined(PIO_UNIT_TESTING) && !defined(UNITY_INCLUDE_DOUBLE)
#define UNITY_INCLUDE_DOUBLE
#include <unity.h>
#endif // PIO_UNIT_TESTING


#endif // NATIVE_TEST_HELPER_H
