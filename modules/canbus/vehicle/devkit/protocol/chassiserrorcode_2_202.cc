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

#include "modules/canbus/vehicle/devkit/protocol/chassiserrorcode_2_202.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Chassiserrorcode2202::Chassiserrorcode2202() {}
const int32_t Chassiserrorcode2202::ID = 0x202;

void Chassiserrorcode2202::Parse(const std::uint8_t* bytes, int32_t length,
                                 ChassisDetail* chassis) const {
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_2_202()
      ->set_chassisrredserrorcode2(chassisrredserrorcode2(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_2_202()
      ->set_chassisrfedserrorcode2(chassisrfedserrorcode2(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_2_202()
      ->set_chassislredserrorcode2(chassislredserrorcode2(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_2_202()
      ->set_chassislfedserrorcode2(chassislfedserrorcode2(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_2_202()
      ->set_chassisrredserrorcode(chassisrredserrorcode(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_2_202()
      ->set_chassisrfedserrorcode(chassisrfedserrorcode(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_2_202()
      ->set_chassislredserrorcode(chassislredserrorcode(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_2_202()
      ->set_chassislfedserrorcode(chassislfedserrorcode(bytes, length));
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'chassisrredserrorcode2', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode2202::chassisrredserrorcode2(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
// 'chassisrfedserrorcode2', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode2202::chassisrfedserrorcode2(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
// 'chassislredserrorcode2', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode2202::chassislredserrorcode2(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
// 'chassislfedserrorcode2', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode2202::chassislfedserrorcode2(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
// 'chassisrredserrorcode', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode2202::chassisrredserrorcode(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
// 'chassisrfedserrorcode', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode2202::chassisrfedserrorcode(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
// 'chassislredserrorcode', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode2202::chassislredserrorcode(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'description': '1', 'is_signed_var': False, 'len':
// 8, 'name': 'chassislfedserrorcode', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Chassiserrorcode2202::chassislfedserrorcode(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
