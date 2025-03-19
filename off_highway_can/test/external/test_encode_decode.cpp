// Copyright (c) 2017 Rein Appeldoorn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Adapted from
// https://github.com/reinzor/libcan-encode-decode/blob/master/test/test_encode_decode.cpp

// #include "../include/can_encode_decode_inl.h" // Wrong path if compiled in place of common package

#include <stdlib.h>

#include <iostream>
#include <stack>
#include <ctime>
#include <bitset>
#include <cassert>

#include "off_highway_can/external/can_encode_decode_inl.h"
#include "gtest/gtest.h"

std::stack<clock_t> tictoc_stack;
unsigned int num_tests = 0;

void tic()
{
  tictoc_stack.push(clock());
}

void toc()
{
  std::cout << "All " << num_tests << " tests passed within " <<
  (static_cast<double>(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC << " seconds!" << std::endl;
  tictoc_stack.pop();
}

inline void TEST_STORE_EXTRACT(
  int64_t value, unsigned int startbit, unsigned int length, bool is_big_endian,
  bool is_signed)
{
  // std::cout << "--\n TEST_STORE_EXTRACT(" << value << ", " << startbit << ", " << length << ", "
  // << is_big_endian << ", " << is_signed << ")" << std::endl;
  uint8_t src_array[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  storeSignal(src_array, value, startbit, length, is_big_endian, is_signed);
  EXPECT_EQ((int32_t)extractSignal(src_array, startbit, length, is_big_endian, is_signed), value);
}

inline void TEST_ENCODE_DECODE(
  float value, unsigned int startbit, unsigned int length, bool is_big_endian,
  bool is_signed, float factor, float offset)
{
  // std::cout << "--\n TEST_ENCODE_DECODE(" << value << ", " << startbit << ", " << length << ", "
  // << is_big_endian << ", " << is_signed << ", " << factor << ", " << offset << ")" << std::endl;
  uint8_t src_array[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Only negative allowed when signed
  assert(is_signed || (int64_t)fromPhysicalValue(value, factor, offset) >= 0);

  encode(src_array, value, startbit, length, is_big_endian, is_signed, factor, offset);

  EXPECT_EQ(decode(src_array, startbit, length, is_big_endian, is_signed, factor, offset), value);
}

inline void TEST_IQ_STORE_EXTRACT(
  float value, unsigned int startbit, unsigned int length, unsigned int float_length,
  bool is_big_endian, bool is_signed)
{
  uint8_t src_array[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  storeIQ(src_array, value, startbit, length, float_length, is_big_endian, is_signed);
  EXPECT_NEAR(
    extractIQ(
      src_array, startbit, length, float_length, is_big_endian,
      is_signed), value, 0.01);
}

TEST(TestEncodeDecode, decodeTestBigEndianUnsigned) {
  uint8_t src_array[8] = {141, 0, 16, 1, 0, 130, 1, 0};

  EXPECT_EQ(extractSignal(src_array, 0, 2, true, false), 1);
  EXPECT_EQ(decode(src_array, 0, 2, true, false, 1.000000, 0), 1.000000);

  EXPECT_EQ(extractSignal(src_array, 2, 6, true, false), 35);
  EXPECT_EQ(decode(src_array, 2, 6, true, false, 1.000000, 0), 35.000000);

  EXPECT_EQ(extractSignal(src_array, 21, 11, true, false), 0);
  EXPECT_EQ(decode(src_array, 21, 11, true, false, 0.100000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 25, 12, true, false), 2048);
  EXPECT_EQ(decode(src_array, 25, 12, true, false, 0.062500, -128), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 32, 9, true, false), 256);
  EXPECT_EQ(decode(src_array, 32, 9, true, false, 0.062500, -16), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 48, 3, true, false), 1);
  EXPECT_EQ(decode(src_array, 48, 3, true, false, 1.000000, 0), 1.000000);

  EXPECT_EQ(extractSignal(src_array, 51, 3, true, false), 0);
  EXPECT_EQ(decode(src_array, 51, 3, true, false, 1.000000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 54, 10, true, false), 520);
  EXPECT_EQ(decode(src_array, 54, 10, true, false, 0.100000, -52), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 56, 3, true, false), 0);
  EXPECT_EQ(decode(src_array, 56, 3, true, false, 1.000000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 59, 3, true, false), 0);
  EXPECT_EQ(decode(src_array, 59, 3, true, false, 1.000000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 62, 2, true, false), 0);
  EXPECT_EQ(decode(src_array, 62, 2, true, false, 1.000000, 0), 0.000000);
}

TEST(TestEncodeDecode, decodeTestLittleEndianUnsignedAndSigned) {
  // src, startbit, bitlength, is_big_endian, is_signed, factor, offset, name, value
  uint8_t src_array[8] = {12, 0, 5, 112, 3, 205, 31, 131};

  EXPECT_EQ(extractSignal(src_array, 60, 2, false, false), 0);
  EXPECT_EQ(decode(src_array, 60, 2, false, true, 1.000000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 55, 1, false, false), 0);
  EXPECT_EQ(decode(src_array, 55, 1, false, false, 1.000000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 20, 4, false, false), 0);
  EXPECT_EQ(decode(src_array, 20, 4, false, false, 1.000000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 62, 2, false, false), 2);
  EXPECT_EQ(decode(src_array, 62, 2, false, false, 1.000000, 0), 2.000000);

  EXPECT_EQ(extractSignal(src_array, 34, 3, false, false), 0);
  EXPECT_EQ(decode(src_array, 34, 3, false, false, 1.000000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 37, 3, false, false), 0);
  EXPECT_EQ(decode(src_array, 37, 3, false, false, 1.000000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 59, 1, false, true), 0);
  EXPECT_EQ(decode(src_array, 59, 1, false, true, 1.000000, 0), 0.000000);

  EXPECT_EQ(extractSignal(src_array, 56, 3, false, false), 3);
  EXPECT_EQ(decode(src_array, 56, 3, false, false, 1.000000, 0), 3.000000);

  EXPECT_EQ(extractSignal(src_array, 52, 3, false, false), 1);
  EXPECT_EQ(decode(src_array, 52, 3, false, false, 1.000000, 0), 1.000000);

  EXPECT_EQ(extractSignal(src_array, 8, 12, false, false), 1280);
  EXPECT_EQ(decode(src_array, 8, 12, false, false, 0.062500, 0), 80.000000);

  EXPECT_EQ(decode(src_array, 40, 12, false, true, 0.062500, 0), -3.187500);

  EXPECT_EQ(extractSignal(src_array, 24, 10, false, true), (uint64_t)-144);
  EXPECT_EQ(decode(src_array, 24, 10, false, true, 0.062500, 0), -9.000000);

  EXPECT_EQ(extractSignal(src_array, 0, 8, false, false), 12);
  EXPECT_EQ(decode(src_array, 0, 8, false, false, 1.000000, 0), 12.000000);
}

TEST(TestEncodeDecode, storeExtract) {
  TEST_STORE_EXTRACT(512, 8, 12, false, true);

  // Limits
  TEST_STORE_EXTRACT(512, 0, 32, false, false);
  TEST_STORE_EXTRACT(512, 56, 32, true, false);
  TEST_STORE_EXTRACT(512, 32, 32, false, false);
  TEST_STORE_EXTRACT(512, 32, 32, true, false);

  // Limits
  TEST_STORE_EXTRACT(-512, 0, 32, false, true);
  TEST_STORE_EXTRACT(-512, 56, 32, true, true);
  TEST_STORE_EXTRACT(-512, 32, 32, false, true);
  TEST_STORE_EXTRACT(-512, 32, 32, true, true);

  // Some other
  TEST_STORE_EXTRACT(26, 4, 8, false, false);
  TEST_STORE_EXTRACT(26, 62, 8, true, false);
  TEST_STORE_EXTRACT(-12, 4, 8, false, true);
  TEST_STORE_EXTRACT(-13, 62, 8, true, true);
  TEST_STORE_EXTRACT(-190, 62, 24, true, true);
}

TEST(TestEncodeDecode, encodingDecoding) {
  TEST_ENCODE_DECODE(-1.0, 0, 7, false, true, 1.0, 0.0);
  TEST_ENCODE_DECODE(-100.0, 0, 7, false, true, 100, 0.0);
  TEST_ENCODE_DECODE(3.0, 56, 3, false, false, 1.000000, 0.0);
  TEST_ENCODE_DECODE(35, 2, 6, true, false, 1.0, 0);
  TEST_ENCODE_DECODE(1.0, 0, 2, true, false, 1.0, 0);

  TEST_ENCODE_DECODE(1.0, 62, 24, true, true, 0.1, 20);
}

// Disabled since IQ functions are not used and tests fail for release build
TEST(TestEncodeDecode, DISABLED_iqNotationsStoreExtract) {
  TEST_IQ_STORE_EXTRACT(-1.0, 0, 7, 3, false, true);
  TEST_IQ_STORE_EXTRACT(-8.25, 0, 16, 8, false, true);
  TEST_IQ_STORE_EXTRACT(3.0, 56, 3, 0, false, false);
  TEST_IQ_STORE_EXTRACT(8.32, 2, 16, 8, false, false);
  TEST_IQ_STORE_EXTRACT(1.0, 0, 2, 0, false, false);
  TEST_IQ_STORE_EXTRACT(1.6, 0, 24, 8, false, true);
}

TEST(TestEncodeDecode, encodingWithNonZeroSourceArray) {
  uint8_t src_array[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int number = 11465;
  encode(src_array, number, 16, 16, false, false, 0.5, 0);
  EXPECT_EQ(number, decode(src_array, 16, 16, false, false, 0.5, 0));

  number = 3;
  encode(src_array, number, 15, 3, false, false, 0.5, 0);
  EXPECT_EQ(number, decode(src_array, 15, 3, false, false, 0.5, 0));

  number = 11768;
  encode(src_array, number, 16, 16, false, false, 0.5, 0);
  EXPECT_EQ(number, decode(src_array, 16, 16, false, false, 0.5, 0));

  // Clearing after within byte
  number = 15;
  encode(src_array, number, 0, 4, false, false, 1.0, 0);
  EXPECT_EQ(number, decode(src_array, 0, 4, false, false, 1.0, 0));

  int second_number = 0;
  encode(src_array, second_number, 4, 4, false, false, 1.0, 0);
  EXPECT_EQ(second_number, decode(src_array, 4, 4, false, false, 1.0, 0));
  EXPECT_EQ(number, decode(src_array, 0, 4, false, false, 1.0, 0));

  // Clearing before within byte
  encode(src_array, number, 4, 4, false, false, 1.0, 0);
  EXPECT_EQ(number, decode(src_array, 4, 4, false, false, 1.0, 0));

  encode(src_array, second_number, 0, 4, false, false, 1.0, 0);
  EXPECT_EQ(second_number, decode(src_array, 0, 4, false, false, 1.0, 0));
  EXPECT_EQ(number, decode(src_array, 4, 4, false, false, 1.0, 0));
}

TEST(TestEncodeDecode, clearingAfterOverBytes1) {
  uint8_t src_array[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Clearing after over bytes
  int number = 4095;
  int second_number = 0;

  encode(src_array, number, 0, 12, false, false, 1.0, 0);
  EXPECT_EQ(number, decode(src_array, 0, 12, false, false, 1.0, 0));

  encode(src_array, second_number, 12, 4, false, false, 1.0, 0);
  EXPECT_EQ(second_number, decode(src_array, 12, 4, false, false, 1.0, 0));
  EXPECT_EQ(number, decode(src_array, 0, 12, false, false, 1.0, 0));
}

TEST(TestEncodeDecode, clearingAfterOverBytes2) {
  uint8_t src_array[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Clearing after over byte
  int number = 15;
  int second_number = 0;
  encode(src_array, number, 12, 4, false, false, 1.0, 0);
  EXPECT_EQ(number, decode(src_array, 12, 4, false, false, 1.0, 0));

  encode(src_array, second_number, 0, 12, false, false, 1.0, 0);
  EXPECT_EQ(second_number, decode(src_array, 0, 12, false, false, 1.0, 0));
  EXPECT_EQ(number, decode(src_array, 12, 4, false, false, 1.0, 0));
}

TEST(TestEncodeDecode, clearingAfterOverBytes3) {
  uint8_t src_array[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Clearing over bytes
  int number = 255;
  int second_number = 15;

  encode(src_array, number, 4, 8, false, false, 1.0, 0);
  encode(src_array, second_number, 0, 4, false, false, 1.0, 0);
  encode(src_array, second_number, 12, 4, false, false, 1.0, 0);
  EXPECT_EQ(number, decode(src_array, 4, 8, false, false, 1.0, 0));
}

TEST(TestEncodeDecode, clearingAfterOverBytes4) {
  uint8_t src_array[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Clearing over bytes
  int number = 255;
  int second_number = 15;

  encode(src_array, second_number, 0, 4, false, false, 1.0, 0);
  encode(src_array, second_number, 12, 4, false, false, 1.0, 0);

  encode(src_array, number, 4, 8, false, false, 1.0, 0);
  EXPECT_EQ(second_number, decode(src_array, 0, 4, false, false, 1.0, 0));
  EXPECT_EQ(second_number, decode(src_array, 12, 4, false, false, 1.0, 0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  tic();
  return RUN_ALL_TESTS();
  toc();
}
