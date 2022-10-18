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

class Vinresp2515 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Vinresp2515();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name':
  // 'VIN15', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vin15(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name':
  // 'VIN14', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vin14(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name':
  // 'VIN13', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vin13(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 8, 'name':
  // 'VIN12', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vin12(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 8, 'name':
  // 'VIN11', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vin11(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 23, 'is_signed_var': False, 'len': 8, 'name':
  // 'VIN10', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vin10(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name':
  // 'VIN09', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vin09(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 8, 'name':
  // 'VIN08', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int vin08(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
