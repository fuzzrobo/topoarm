// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Modified by: Nagashima

#ifndef TOPOARM_HARDWARE__VISIBILITY_CONTROL_H_
#define TOPOARM_HARDWARE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TOPOARM_HARDWARE_EXPORT __attribute__((dllexport))
#define TOPOARM_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define TOPOARM_HARDWARE_EXPORT __declspec(dllexport)
#define TOPOARM_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef TOPOARM_HARDWARE_BUILDING_DLL
#define TOPOARM_HARDWARE_PUBLIC TOPOARM_HARDWARE_EXPORT
#else
#define TOPOARM_HARDWARE_PUBLIC TOPOARM_HARDWARE_IMPORT
#endif
#define TOPOARM_HARDWARE_PUBLIC_TYPE TOPOARM_HARDWARE_PUBLIC
#define TOPOARM_HARDWARE_LOCAL
#else
#define TOPOARM_HARDWARE_EXPORT __attribute__((visibility("default")))
#define TOPOARM_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define TOPOARM_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define TOPOARM_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define TOPOARM_HARDWARE_PUBLIC
#define TOPOARM_HARDWARE_LOCAL
#endif
#define TOPOARM_HARDWARE_PUBLIC_TYPE
#endif

#endif  // TOPOARM_HARDWARE__VISIBILITY_CONTROL_H_
