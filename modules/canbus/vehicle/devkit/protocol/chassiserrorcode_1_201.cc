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

#include "modules/canbus/vehicle/devkit/protocol/chassiserrorcode_1_201.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::drivers::canbus::Byte;

Chassiserrorcode1201::Chassiserrorcode1201() {}
const int32_t Chassiserrorcode1201::ID = 0x201;

void Chassiserrorcode1201::Parse(const std::uint8_t* bytes, int32_t length,
                                 ChassisDetail* chassis) const {
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_1_201()
      ->set_chassisbmserrorcode(chassisbmserrorcode(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_1_201()
      ->set_chassisbackepserrorcode(chassisbackepserrorcode(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_1_201()
      ->set_chassisedstype(chassisedstype(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_1_201()
      ->set_chassisfrontepserrorcode(chassisfrontepserrorcode(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_1_201()
      ->set_chassisepstype(chassisepstype(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_1_201()
      ->set_chassisepbtype(chassisepbtype(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_1_201()
      ->set_chassisepberrorcode(chassisepberrorcode(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_1_201()
      ->set_chassisebstype(chassisebstype(bytes, length));
  chassis->mutable_devkit()
      ->mutable_chassiserrorcode_1_201()
      ->set_chassisebserrorcode(chassisebserrorcode(bytes, length));
}

// config detail: {'bit': 24, 'description': '1', 'is_signed_var': False, 'len':
// 8, 'name': 'chassisbmserrorcode', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Chassiserrorcode1201::chassisbmserrorcode(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'description': '1', 'is_signed_var': False, 'len':
// 8, 'name': 'chassisbackepserrorcode', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Chassiserrorcode1201::chassisbackepserrorcode(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 14, 'is_signed_var': False, 'len': 2, 'name':
// 'chassisedstype', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode1201::chassisedstype(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'description': '1', 'is_signed_var': False, 'len':
// 8, 'name': 'chassisfrontepserrorcode', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Chassiserrorcode1201::chassisfrontepserrorcode(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 2, 'name':
// 'chassisepstype', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode1201::chassisepstype(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name':
// 'chassisepbtype', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode1201::chassisepbtype(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'description': '1', 'is_signed_var': False, 'len':
// 8, 'name': 'chassisepberrorcode', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Chassiserrorcode1201::chassisepberrorcode(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 2, 'name':
// 'chassisebstype', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassiserrorcode1201::chassisebstype(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'description': '1', 'is_signed_var': False, 'len':
// 8, 'name': 'chassisebserrorcode', 'offset': 0.0, 'order': 'intel',
// 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type':
// 'int'}
int Chassiserrorcode1201::chassisebserrorcode(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
