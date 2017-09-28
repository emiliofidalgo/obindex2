/**
* This file is part of obindex2.
*
* Copyright (C) 2017 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
* For more information see: <http://dmi.uib.es/~egarcia>
*
* obindex2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* obindex2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with bimos. If not, see <http://www.gnu.org/licenses/>.
*/

#include "obindex2/binary_descriptor.h"

namespace obindex2 {

BinaryDescriptor::BinaryDescriptor(const int nbits) {
  bitset_.resize(nbits);
  bitset_.reset();
}

BinaryDescriptor::BinaryDescriptor(const cv::Mat& desc) {
  assert(desc.type() == CV_8U);

  // Initializing the bitset
  bitset_.resize(desc.cols * 8);
  bitset_.reset();

  // Creating the descriptor
  unsigned curr_bit = 0;
  const unsigned char* chars = desc.ptr<unsigned char>(0);

  for (int curr_char = 0; curr_char < desc.cols; curr_char++) {
    // Getting the next character of the descriptor
    unsigned char ch = chars[curr_char];

    // Validate each bit of the character
    for (int i = 0; i < 8; i++) {
      if (ch & (1 << (7 - i))) {
        bitset_.set(curr_bit);
      }
      curr_bit++;
    }
  }
}

cv::Mat BinaryDescriptor::toCvMat() {
  // Compute the size in columns
  int ncols = static_cast<int>(size() / 8.0);

  cv::Mat m = cv::Mat::zeros(1, ncols, CV_8U);
  unsigned char* d = m.ptr<unsigned char>(0);

  int curr_bit = 0;
  unsigned char curr_char = 0;
  unsigned char ch = 0;
  while (curr_bit < size()) {
    for (int i = 0; i < 8; i++) {
      if (bitset_.test(curr_bit + i)) {
        ch |= (1 << (7 - i));
      }
    }

    // Storing the current character
    d[curr_char] = ch;

    // Updating the loop variables
    curr_char++;
    curr_bit += 8;
    ch = 0;
  }

  return m.clone();
}

std::string BinaryDescriptor::toString() {
  std::string st;
  to_string(bitset_, st);
  return st;
}

}  // namespace obindex2
