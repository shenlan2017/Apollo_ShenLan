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

class Chassiserrorcode1201 : public ::apollo::drivers::canbus::ProtocolData<
                                 ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Chassiserrorcode1201();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 24, 'description': '1', 'is_signed_var': False,
  // 'len': 8, 'name': 'ChassisBmsErrorCode', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int chassisbmserrorcode(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 56, 'description': '1', 'is_signed_var': False,
  // 'len': 8, 'name': 'ChassisBackEpsErrorCode', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
  // 'type': 'int'}
  int chassisbackepserrorcode(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 14, 'is_signed_var': False, 'len': 2, 'name':
  // 'ChassisEdsType', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisedstype(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'description': '1', 'is_signed_var': False,
  // 'len': 8, 'name': 'ChassisFrontEpsErrorCode', 'offset': 0.0, 'order':
  // 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0,
  // 'type': 'int'}
  int chassisfrontepserrorcode(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 2, 'name':
  // 'ChassisEpsType', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisepstype(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name':
  // 'ChassisEpbType', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisepbtype(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'description': '1', 'is_signed_var': False,
  // 'len': 8, 'name': 'ChassisEpbErrorCode', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int chassisepberrorcode(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 2, 'name':
  // 'ChassisEbsType', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int chassisebstype(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'description': '1', 'is_signed_var': False,
  // 'len': 8, 'name': 'ChassisEbsErrorCode', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'int'}
  int chassisebserrorcode(const std::uint8_t* bytes,
                          const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
