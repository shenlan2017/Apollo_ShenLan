/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus/vehicle/devkit/protocol/chassiserrorcode_200.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Chassiserrorcode200::Chassiserrorcode200() {}
const int32_t Chassiserrorcode200::ID = 0x200;

void Chassiserrorcode200::Parse(const std::uint8_t* bytes, int32_t length,
                                ChassisDetail* chassis) const {
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_bms_communicationfault(bms_communicationfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassisrredsfault(chassisrredsfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassisrfedsfault(chassisrfedsfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassislredsfault(chassislredsfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassislfedsfault(chassislfedsfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassisfrontepsfault(chassisfrontepsfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassisebsfault(chassisebsfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassisbackepsfault(chassisbackepsfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassisemergencysta(chassisemergencysta(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_leadacidbatterylow(leadacidbatterylow(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_controlercommunicationfault(
          controlercommunicationfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassiscrashsta(chassiscrashsta(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_eps_rearcommunicationfault(
          eps_rearcommunicationfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_eps_frontcommunicationfault(
          eps_frontcommunicationfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_eds_rrcommunicationfault(eds_rrcommunicationfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_eds_rfcommunicationfault(eds_rfcommunicationfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_eds_lrcommunicationfault(eds_lrcommunicationfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_eds_lfcommunicationfault(eds_lfcommunicationfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_ebs_communicationfault(ebs_communicationfault(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_200()
      ->set_chassisvcuothererr(chassisvcuothererr(bytes, length));
}

// config detail: {'bit': 7, 'description': '1', 'enum': {0:
// 'BMS_COMMUNICATIONFAULT_CONNECTION', 1:
// 'BMS_COMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len': 1,
// 'name': 'bms_communicationfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::Bms_communicationfaultType
Chassiserrorcode200::bms_communicationfault(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Chassiserrorcode_200::Bms_communicationfaultType ret =
      static_cast<Chassiserrorcode_200::Bms_communicationfaultType>(x);
  return ret;
}

// config detail: {'bit': 38, 'description': '1', 'enum': {0:
// 'CHASSISRREDSFAULT_NO_ERROR', 1: 'CHASSISRREDSFAULT_ERROR'}, 'is_signed_var':
// False, 'len': 2, 'name': 'chassisrredsfault', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'enum'}
Chassiserrorcode_200::ChassisrredsfaultType
Chassiserrorcode200::chassisrredsfault(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 2);

  Chassiserrorcode_200::ChassisrredsfaultType ret =
      static_cast<Chassiserrorcode_200::ChassisrredsfaultType>(x);
  return ret;
}

// config detail: {'bit': 34, 'description': '1', 'enum': {0:
// 'CHASSISRFEDSFAULT_NO_ERROR', 1: 'CHASSISRFEDSFAULT_ERROR'}, 'is_signed_var':
// False, 'len': 2, 'name': 'chassisrfedsfault', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'enum'}
Chassiserrorcode_200::ChassisrfedsfaultType
Chassiserrorcode200::chassisrfedsfault(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 2);

  Chassiserrorcode_200::ChassisrfedsfaultType ret =
      static_cast<Chassiserrorcode_200::ChassisrfedsfaultType>(x);
  return ret;
}

// config detail: {'bit': 36, 'description': '1', 'enum': {0:
// 'CHASSISLREDSFAULT_NO_ERROR', 1: 'CHASSISLREDSFAULT_ERROR'}, 'is_signed_var':
// False, 'len': 2, 'name': 'chassislredsfault', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'enum'}
Chassiserrorcode_200::ChassislredsfaultType
Chassiserrorcode200::chassislredsfault(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 2);

  Chassiserrorcode_200::ChassislredsfaultType ret =
      static_cast<Chassiserrorcode_200::ChassislredsfaultType>(x);
  return ret;
}

// config detail: {'bit': 32, 'description': '1', 'enum': {0:
// 'CHASSISLFEDSFAULT_NO_ERROR', 1: 'CHASSISLFEDSFAULT_ERROR'}, 'is_signed_var':
// False, 'len': 2, 'name': 'chassislfedsfault', 'offset': 0.0, 'order':
// 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
// 'type': 'enum'}
Chassiserrorcode_200::ChassislfedsfaultType
Chassiserrorcode200::chassislfedsfault(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Chassiserrorcode_200::ChassislfedsfaultType ret =
      static_cast<Chassiserrorcode_200::ChassislfedsfaultType>(x);
  return ret;
}

// config detail: {'bit': 40, 'description': '1', 'enum': {0:
// 'CHASSISFRONTEPSFAULT_NO_ERROR', 1: 'CHASSISFRONTEPSFAULT_ERROR'},
// 'is_signed_var': False, 'len': 2, 'name': 'chassisfrontepsfault', 'offset':
// 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
Chassiserrorcode_200::ChassisfrontepsfaultType
Chassiserrorcode200::chassisfrontepsfault(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  Chassiserrorcode_200::ChassisfrontepsfaultType ret =
      static_cast<Chassiserrorcode_200::ChassisfrontepsfaultType>(x);
  return ret;
}

// config detail: {'bit': 44, 'description': '1', 'enum': {0:
// 'CHASSISEBSFAULT_NO_ERROR', 1: 'CHASSISEBSFAULT_ERROR'}, 'is_signed_var':
// False, 'len': 2, 'name': 'chassisebsfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::ChassisebsfaultType Chassiserrorcode200::chassisebsfault(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 2);

  Chassiserrorcode_200::ChassisebsfaultType ret =
      static_cast<Chassiserrorcode_200::ChassisebsfaultType>(x);
  return ret;
}

// config detail: {'bit': 42, 'description': '1', 'enum': {0:
// 'CHASSISBACKEPSFAULT_NO_ERROR', 1: 'CHASSISBACKEPSFAULT_ERROR'},
// 'is_signed_var': False, 'len': 2, 'name': 'chassisbackepsfault', 'offset':
// 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
Chassiserrorcode_200::ChassisbackepsfaultType
Chassiserrorcode200::chassisbackepsfault(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 2);

  Chassiserrorcode_200::ChassisbackepsfaultType ret =
      static_cast<Chassiserrorcode_200::ChassisbackepsfaultType>(x);
  return ret;
}

// config detail: {'bit': 10, 'description': '1', 'is_signed_var': False, 'len':
// 1, 'name': 'chassisemergencysta', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Chassiserrorcode200::chassisemergencysta(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 11, 'description': '1', 'enum': {0:
// 'LEADACIDBATTERYLOW_NO_ERROR', 1: 'LEADACIDBATTERYLOW_ERROR'},
// 'is_signed_var': False, 'len': 1, 'name': 'leadacidbatterylow', 'offset':
// 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
// 'precision': 1.0, 'type': 'enum'}
Chassiserrorcode_200::LeadacidbatterylowType
Chassiserrorcode200::leadacidbatterylow(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  Chassiserrorcode_200::LeadacidbatterylowType ret =
      static_cast<Chassiserrorcode_200::LeadacidbatterylowType>(x);
  return ret;
}

// config detail: {'bit': 8, 'description': '1', 'enum': {0:
// 'CONTROLERCOMMUNICATIONFAULT_CONNECTION', 1:
// 'CONTROLERCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len':
// 1, 'name': 'controlercommunicationfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::ControlercommunicationfaultType
Chassiserrorcode200::controlercommunicationfault(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Chassiserrorcode_200::ControlercommunicationfaultType ret =
      static_cast<Chassiserrorcode_200::ControlercommunicationfaultType>(x);
  return ret;
}

// config detail: {'bit': 9, 'description': '1', 'is_signed_var': False, 'len':
// 1, 'name': 'chassiscrashsta', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'bool'}
bool Chassiserrorcode200::chassiscrashsta(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 6, 'description': '1', 'enum': {0:
// 'EPS_REARCOMMUNICATIONFAULT_CONNECTION', 1:
// 'EPS_REARCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len':
// 1, 'name': 'eps_rearcommunicationfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::Eps_rearcommunicationfaultType
Chassiserrorcode200::eps_rearcommunicationfault(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  Chassiserrorcode_200::Eps_rearcommunicationfaultType ret =
      static_cast<Chassiserrorcode_200::Eps_rearcommunicationfaultType>(x);
  return ret;
}

// config detail: {'bit': 5, 'description': '1', 'enum': {0:
// 'EPS_FRONTCOMMUNICATIONFAULT_CONNECTION', 1:
// 'EPS_FRONTCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len':
// 1, 'name': 'eps_frontcommunicationfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::Eps_frontcommunicationfaultType
Chassiserrorcode200::eps_frontcommunicationfault(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  Chassiserrorcode_200::Eps_frontcommunicationfaultType ret =
      static_cast<Chassiserrorcode_200::Eps_frontcommunicationfaultType>(x);
  return ret;
}

// config detail: {'bit': 3, 'description': '1', 'enum': {0:
// 'EDS_RRCOMMUNICATIONFAULT_CONNECTION', 1:
// 'EDS_RRCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len': 1,
// 'name': 'eds_rrcommunicationfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::Eds_rrcommunicationfaultType
Chassiserrorcode200::eds_rrcommunicationfault(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  Chassiserrorcode_200::Eds_rrcommunicationfaultType ret =
      static_cast<Chassiserrorcode_200::Eds_rrcommunicationfaultType>(x);
  return ret;
}

// config detail: {'bit': 2, 'description': '1', 'enum': {0:
// 'EDS_RFCOMMUNICATIONFAULT_CONNECTION', 1:
// 'EDS_RFCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len': 1,
// 'name': 'eds_rfcommunicationfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::Eds_rfcommunicationfaultType
Chassiserrorcode200::eds_rfcommunicationfault(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  Chassiserrorcode_200::Eds_rfcommunicationfaultType ret =
      static_cast<Chassiserrorcode_200::Eds_rfcommunicationfaultType>(x);
  return ret;
}

// config detail: {'bit': 1, 'description': '1', 'enum': {0:
// 'EDS_LRCOMMUNICATIONFAULT_CONNECTION', 1:
// 'EDS_LRCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len': 1,
// 'name': 'eds_lrcommunicationfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::Eds_lrcommunicationfaultType
Chassiserrorcode200::eds_lrcommunicationfault(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Chassiserrorcode_200::Eds_lrcommunicationfaultType ret =
      static_cast<Chassiserrorcode_200::Eds_lrcommunicationfaultType>(x);
  return ret;
}

// config detail: {'bit': 0, 'description': '1', 'enum': {0:
// 'EDS_LFCOMMUNICATIONFAULT_CONNECTION', 1:
// 'EDS_LFCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len': 1,
// 'name': 'eds_lfcommunicationfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::Eds_lfcommunicationfaultType
Chassiserrorcode200::eds_lfcommunicationfault(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Chassiserrorcode_200::Eds_lfcommunicationfaultType ret =
      static_cast<Chassiserrorcode_200::Eds_lfcommunicationfaultType>(x);
  return ret;
}

// config detail: {'bit': 4, 'description': '1', 'enum': {0:
// 'EBS_COMMUNICATIONFAULT_CONNECTION', 1:
// 'EBS_COMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len': 1,
// 'name': 'ebs_communicationfault', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'enum'}
Chassiserrorcode_200::Ebs_communicationfaultType
Chassiserrorcode200::ebs_communicationfault(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  Chassiserrorcode_200::Ebs_communicationfaultType ret =
      static_cast<Chassiserrorcode_200::Ebs_communicationfaultType>(x);
  return ret;
}

// config detail: {'bit': 16, 'description': '1', 'is_signed_var': False, 'len':
// 8, 'name': 'chassisvcuothererr', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Chassiserrorcode200::chassisvcuothererr(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
