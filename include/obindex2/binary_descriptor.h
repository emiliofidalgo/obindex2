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

#ifndef INCLUDE_OBINDEX2_BINARY_DESCRIPTOR_H_
#define INCLUDE_OBINDEX2_BINARY_DESCRIPTOR_H_

#include <string>
#include <sstream>

#include <boost/dynamic_bitset.hpp>
#include <opencv2/opencv.hpp>

namespace obindex2 {

typedef boost::dynamic_bitset<> Bitset;

class BinaryDescriptor {
 public:
  // Constructors
  explicit BinaryDescriptor(const int nbits = 256);
  explicit BinaryDescriptor(const cv::Mat& desc);

  // Methods
  inline void set(int nbit) {
    bitset_.set(nbit);
  }

  inline void reset(int nbit) {
    bitset_.reset(nbit);
  }

  inline int size() {
    return bitset_.size();
  }

  inline static double distHamming(const BinaryDescriptor& a,
                                   const BinaryDescriptor& b) {
    return static_cast<double>((a.bitset_^b.bitset_).count());
  }

  // Operator overloading
  inline bool operator==(const BinaryDescriptor& d) {
    int distance = (bitset_^d.bitset_).count();
    return distance == 0;
  }

  inline bool operator!=(const BinaryDescriptor& d) {
    int distance = (bitset_^d.bitset_).count();
    return distance != 0;
  }

  inline BinaryDescriptor& operator=(const BinaryDescriptor& other) {
    assert(size() == other.size());
    bitset_.clear();
    bitset_ = other.bitset_;
    return *this;
  }

  cv::Mat toCvMat();
  std::string toString();

  // For simplicity, we made it public, but you should use the methods.
  Bitset bitset_;
};

}  // namespace obindex2

#endif  // INCLUDE_OBINDEX2_BINARY_DESCRIPTOR_H_
