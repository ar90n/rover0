#include "utest.h"

#include "endian_utils.hpp"

UTEST(endian_utils, test_pack_int16_be)
{
  uint8_t buffer[2] = {0x12, 0x34};
  ASSERT_EQ(pack_int16_be(buffer), 0x1234);
}

UTEST(endian_utils, test_pack_uint32_be)
{
  uint8_t buffer[4] = {0x12, 0x34, 0x56, 0x78};
  ASSERT_EQ(pack_uint32_be(buffer), 0x12345678);
}

UTEST(endian_utils, test_unpack_int16_be)
{
  auto const result = unpack_int16_be(0x1234);
  ASSERT_EQ(result[0], 0x12);
  ASSERT_EQ(result[1], 0x34);
}

UTEST(endian_utils, test_unpack_uint32_be)
{
  auto const result = unpack_uint32_be(0x12345678);
  ASSERT_EQ(result[0], 0x12);
  ASSERT_EQ(result[1], 0x34);
  ASSERT_EQ(result[2], 0x56);
  ASSERT_EQ(result[3], 0x78);
}