#pragma once
#include <new>
#include "odriveS1.hpp"
#include "mpu6050.hpp"

extern ODRIVES1 *odriveS1Handle;
extern MPU6050 *mpu6050Handle;

void initDrivers();
