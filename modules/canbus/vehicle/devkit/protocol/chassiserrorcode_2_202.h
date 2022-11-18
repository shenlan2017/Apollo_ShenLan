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

class Chassiserrorcode2202 : public ::apollo::drivers::canbus::ProtocolData<
                                 ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Chassiserrorcode2202();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'ChassisRrEdsErrorCode2', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int chassisrredserrorcode2(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
  // 'ChassisRfEdsErrorCode2', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int chassisrfedserrorcode2(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
  // 'ChassisLrEdsErrorCode2', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int chassislredserrorcode2(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
  // 'ChassisLfEdsErrorCode2', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int chassislfedserrorcode2(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'ChassisRrEdsErrorCode', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisrredserrorcode(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
  // 'ChassisRfEdsErrorCode', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisrfedserrorcode(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
  // 'ChassisLrEdsErrorCode', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassislredserrorcode(const std::uint8_t* bytes,
                            const int32_t length) const;

  // config detail: {'bit': 0, 'description': '1', 'is_signed_var': False,
  // 'len': 8, 'name': 'ChassisLfEdsErrorCode', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int chassislfedserrorcode(const std::uint8_t* bytes,
                            const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
