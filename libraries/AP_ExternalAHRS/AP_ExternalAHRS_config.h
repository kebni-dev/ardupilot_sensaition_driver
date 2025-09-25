/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifdef HAL_EXTERNAL_AHRS_ENABLED
#error HAL_EXTERNAL_AHRS_ENABLED has been renamed to AP_EXTERNAL_AHRS_ENABLED
#endif

#ifndef AP_EXTERNAL_AHRS_ENABLED
#define AP_EXTERNAL_AHRS_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 2048
#endif

#ifndef AP_EXTERNAL_AHRS_BACKEND_DEFAULT_ENABLED
#define AP_EXTERNAL_AHRS_BACKEND_DEFAULT_ENABLED AP_EXTERNAL_AHRS_ENABLED
#endif

#ifndef AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED
#define AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED AP_EXTERNAL_AHRS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED
#define AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED AP_EXTERNAL_AHRS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_MICROSTRAIN_ENABLED
#define AP_MICROSTRAIN_ENABLED AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED || AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED
#endif

#ifndef AP_EXTERNAL_AHRS_VECTORNAV_ENABLED
#define AP_EXTERNAL_AHRS_VECTORNAV_ENABLED AP_EXTERNAL_AHRS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
#define AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED AP_EXTERNAL_AHRS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_EXTERNAL_AHRS_SENSAITION_ENABLED
#define AP_EXTERNAL_AHRS_SENSAITION_ENABLED AP_EXTERNAL_AHRS_BACKEND_DEFAULT_ENABLED
#endif