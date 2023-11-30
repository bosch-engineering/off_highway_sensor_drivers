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
//

#ifndef CAN_ENCODE_DECODE_INL_H_
#define CAN_ENCODE_DECODE_INL_H_

#include <stdint.h>  //uint typedefinitions, non-rtw!
#include <assert.h>
#include <math.h>

#define MASK64(nbits) ((0xffffffffffffffff) >> (64 - nbits))


inline float toPhysicalValue(uint64_t target, float factor, float offset, bool is_signed)
{
  if (is_signed)
    return ((int64_t)target) * factor + offset;
  else
    return target * factor + offset;
}

inline uint64_t fromPhysicalValue(float physical_value, float factor, float offset)
{
  return (int64_t)((physical_value - offset) / factor);
}

inline void clearBits(uint8_t* target_byte, uint8_t* bits_to_clear, const uint8_t startbit, const uint8_t length)
{
  for (uint8_t i = startbit; i < length + startbit; ++i)
  {
    *target_byte &= ~(1UL << i);
    *bits_to_clear -= 1;
  }
}

inline void storeSignal(uint8_t* frame, uint64_t value, const uint8_t startbit, const uint8_t length,
                        bool is_big_endian, bool is_signed)
{
  uint8_t start_byte = startbit / 8;
  uint8_t startbit_in_byte = startbit % 8;
  uint8_t end_byte = 0;
  int8_t count = 0;
  uint8_t current_target_length = (8 - startbit_in_byte);
  uint8_t bits_to_clear = length;

  // Mask the value
  value &= MASK64(length);

  // Write bits of startbyte
  clearBits(&frame[start_byte], &bits_to_clear, startbit_in_byte, current_target_length > length ? length : current_target_length);
  frame[start_byte] |= value << startbit_in_byte;

  // Write residual bytes
  if (is_big_endian)  // Motorola (big endian)
  {
    end_byte = (start_byte * 8 + 8 - startbit_in_byte - length) / 8;

    for (count = start_byte - 1; count >= end_byte; count--)
    {
      clearBits(&frame[count], &bits_to_clear, 0, bits_to_clear >= 8 ? 8 : bits_to_clear);
      frame[count] |= value >> current_target_length;
      current_target_length += 8;
    }
  }
  else  // Intel (little endian)
  {
    end_byte = (startbit + length - 1) / 8;

    for (count = start_byte + 1; count <= end_byte; count++)
    {
      clearBits(&frame[count], &bits_to_clear, 0, bits_to_clear >= 8 ? 8 : bits_to_clear);
      frame[count] |= value >> current_target_length;
      current_target_length += 8;
    }
  }
}

inline uint64_t extractSignal(const uint8_t* frame, const uint8_t startbit, const uint8_t length, bool is_big_endian,
                              bool is_signed)
{
  uint8_t start_byte = startbit / 8;
  uint8_t startbit_in_byte = startbit % 8;
  uint8_t current_target_length = (8 - startbit_in_byte);
  uint8_t end_byte = 0;
  int8_t count = 0;

  // Write first bits to target
  uint64_t target = frame[start_byte] >> startbit_in_byte;

  // Write residual bytes
  if (is_big_endian)  // Motorola (big endian)
  {
    end_byte = (start_byte * 8 + 8 - startbit_in_byte - length) / 8;

    for (count = start_byte - 1; count >= end_byte; count--)
    {
      target |= frame[count] << current_target_length;
      current_target_length += 8;
    }
  }
  else  // Intel (little endian)
  {
    end_byte = (startbit + length - 1) / 8;

    for (count = start_byte + 1; count <= end_byte; count++)
    {
      target |= frame[count] << current_target_length;
      current_target_length += 8;
    }
  }

  // Mask value
  target &= MASK64(length);

  // perform sign extension
  if (is_signed)
  {
    int64_t msb_sign_mask = 1 << (length - 1);
    target = ((int64_t)target ^ msb_sign_mask) - msb_sign_mask;
  }

  return target;
}

// For Vector CAN DB files https://vector.com/vi_candb_en.html

inline float decode(const uint8_t* frame, const uint16_t startbit, const uint16_t length, bool is_big_endian,
                    bool is_signed, float factor, float offset)
{
  return toPhysicalValue(extractSignal(frame, startbit, length, is_big_endian, is_signed), factor, offset, is_signed);
}

inline void encode(uint8_t* frame, const float value, const uint16_t startbit, const uint16_t length,
                   bool is_big_endian, bool is_signed, float factor, float offset)
{
  storeSignal(frame, fromPhysicalValue(value, factor, offset), startbit, length, is_big_endian, is_signed);
}

// Texas instruments IQ notation https://en.wikipedia.org/wiki/Q_(number_format)

inline double extractIQ(const uint8_t* frame, uint8_t start, uint8_t length, uint8_t float_length, bool is_big_endian,
                        bool is_signed)
{
  return (int64_t) extractSignal(frame, start, length, is_big_endian, is_signed) / pow(2, float_length);
}

inline void storeIQ(uint8_t* frame, double value, uint8_t start, uint8_t length, uint8_t float_length,
                    bool is_big_endian, bool is_signed)
{
  storeSignal(frame, value * pow(2, float_length), start, length, is_big_endian, is_signed);
}

#endif
