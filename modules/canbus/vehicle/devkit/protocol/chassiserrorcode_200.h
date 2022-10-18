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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace devkit {

class Chassiserrorcode200 : public ::apollo::drivers::canbus::ProtocolData<
                                ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Chassiserrorcode200();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 7, 'description': '1', 'enum': {0:
  // 'BMS_COMMUNICATIONFAULT_CONNECTION', 1:
  // 'BMS_COMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len': 1,
  // 'name': 'BMS_CommunicationFault', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Chassiserrorcode_200::Bms_communicationfaultType bms_communicationfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 38, 'description': '1', 'enum': {0:
  // 'CHASSISRREDSFAULT_NO_ERROR', 1: 'CHASSISRREDSFAULT_ERROR'},
  // 'is_signed_var': False, 'len': 2, 'name': 'ChassisRrEdsFault', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Chassiserrorcode_200::ChassisrredsfaultType chassisrredsfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 34, 'description': '1', 'enum': {0:
  // 'CHASSISRFEDSFAULT_NO_ERROR', 1: 'CHASSISRFEDSFAULT_ERROR'},
  // 'is_signed_var': False, 'len': 2, 'name': 'ChassisRfEdsFault', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Chassiserrorcode_200::ChassisrfedsfaultType chassisrfedsfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 36, 'description': '1', 'enum': {0:
  // 'CHASSISLREDSFAULT_NO_ERROR', 1: 'CHASSISLREDSFAULT_ERROR'},
  // 'is_signed_var': False, 'len': 2, 'name': 'ChassisLrEdsFault', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Chassiserrorcode_200::ChassislredsfaultType chassislredsfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': '1', 'enum': {0:
  // 'CHASSISLFEDSFAULT_NO_ERROR', 1: 'CHASSISLFEDSFAULT_ERROR'},
  // 'is_signed_var': False, 'len': 2, 'name': 'ChassisLfEdsFault', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Chassiserrorcode_200::ChassislfedsfaultType chassislfedsfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'description': '1', 'enum': {0:
  // 'CHASSISFRONTEPSFAULT_NO_ERROR', 1: 'CHASSISFRONTEPSFAULT_ERROR'},
  // 'is_signed_var': False, 'len': 2, 'name': 'ChassisFrontEpsFault', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Chassiserrorcode_200::ChassisfrontepsfaultType chassisfrontepsfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 44, 'description': '1', 'enum': {0:
  // 'CHASSISEBSFAULT_NO_ERROR', 1: 'CHASSISEBSFAULT_ERROR'}, 'is_signed_var':
  // False, 'len': 2, 'name': 'ChassisEbsFault', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
  // 'type': 'enum'}
  Chassiserrorcode_200::ChassisebsfaultType chassisebsfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 42, 'description': '1', 'enum': {0:
  // 'CHASSISBACKEPSFAULT_NO_ERROR', 1: 'CHASSISBACKEPSFAULT_ERROR'},
  // 'is_signed_var': False, 'len': 2, 'name': 'ChassisBackEpsFault', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Chassiserrorcode_200::ChassisbackepsfaultType chassisbackepsfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 10, 'description': '1', 'is_signed_var': False,
  // 'len': 1, 'name': 'ChassisEmergencySta', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool chassisemergencysta(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 11, 'description': '1', 'enum': {0:
  // 'LEADACIDBATTERYLOW_NO_ERROR', 1: 'LEADACIDBATTERYLOW_ERROR'},
  // 'is_signed_var': False, 'len': 1, 'name': 'LeadacidBatteryLow', 'offset':
  // 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '',
  // 'precision': 1.0, 'type': 'enum'}
  Chassiserrorcode_200::LeadacidbatterylowType leadacidbatterylow(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'description': '1', 'enum': {0:
  // 'CONTROLERCOMMUNICATIONFAULT_CONNECTION', 1:
  // 'CONTROLERCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False,
  // 'len': 1, 'name': 'ControlerCommunicationFault', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
  // 'type': 'enum'}
  Chassiserrorcode_200::ControlercommunicationfaultType
  controlercommunicationfault(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 9, 'description': '1', 'is_signed_var': False,
  // 'len': 1, 'name': 'ChassisCrashSta', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool chassiscrashsta(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 6, 'description': '1', 'enum': {0:
  // 'EPS_REARCOMMUNICATIONFAULT_CONNECTION', 1:
  // 'EPS_REARCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len':
  // 1, 'name': 'EPS_RearCommunicationFault', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Chassiserrorcode_200::Eps_rearcommunicationfaultType
  eps_rearcommunicationfault(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 5, 'description': '1', 'enum': {0:
  // 'EPS_FRONTCOMMUNICATIONFAULT_CONNECTION', 1:
  // 'EPS_FRONTCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False,
  // 'len': 1, 'name': 'EPS_FrontCommunicationFault', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
  // 'type': 'enum'}
  Chassiserrorcode_200::Eps_frontcommunicationfaultType
  eps_frontcommunicationfault(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 3, 'description': '1', 'enum': {0:
  // 'EDS_RRCOMMUNICATIONFAULT_CONNECTION', 1:
  // 'EDS_RRCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len':
  // 1, 'name': 'EDS_RrCommunicationFault', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Chassiserrorcode_200::Eds_rrcommunicationfaultType eds_rrcommunicationfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 2, 'description': '1', 'enum': {0:
  // 'EDS_RFCOMMUNICATIONFAULT_CONNECTION', 1:
  // 'EDS_RFCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len':
  // 1, 'name': 'EDS_RfCommunicationFault', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Chassiserrorcode_200::Eds_rfcommunicationfaultType eds_rfcommunicationfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 1, 'description': '1', 'enum': {0:
  // 'EDS_LRCOMMUNICATIONFAULT_CONNECTION', 1:
  // 'EDS_LRCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len':
  // 1, 'name': 'EDS_LrCommunicationFault', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Chassiserrorcode_200::Eds_lrcommunicationfaultType eds_lrcommunicationfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'description': '1', 'enum': {0:
  // 'EDS_LFCOMMUNICATIONFAULT_CONNECTION', 1:
  // 'EDS_LFCOMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len':
  // 1, 'name': 'EDS_LfCommunicationFault', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Chassiserrorcode_200::Eds_lfcommunicationfaultType eds_lfcommunicationfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 4, 'description': '1', 'enum': {0:
  // 'EBS_COMMUNICATIONFAULT_CONNECTION', 1:
  // 'EBS_COMMUNICATIONFAULT_DISCONNECTION'}, 'is_signed_var': False, 'len': 1,
  // 'name': 'EBS_CommunicationFault', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'enum'}
  Chassiserrorcode_200::Ebs_communicationfaultType ebs_communicationfault(
      const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'description': '1', 'is_signed_var': False,
  // 'len': 8, 'name': 'ChassisVcuOtherErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int chassisvcuothererr(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
