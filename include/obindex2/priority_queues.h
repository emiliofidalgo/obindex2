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

#ifndef INCLUDE_OBINDEX2_PRIORITY_QUEUES_H_
#define INCLUDE_OBINDEX2_PRIORITY_QUEUES_H_

#include <queue>
#include <string>
#include <sstream>
#include <vector>

#include "obindex2/binary_descriptor.h"
#include "obindex2/binary_tree_node.h"

namespace obindex2 {

struct PQItemNode {
 public:
  inline explicit PQItemNode(const double d,
                             const unsigned id,
                             BinaryTreeNode* n) :
    dist(d),
    tree_id(id),
    node(n) {}

  double dist;
  unsigned tree_id;
  BinaryTreeNode* node;
};

class CompareNode {
 public:
  inline bool operator()(const PQItemNode& a, const PQItemNode& b) {
    return a.dist > b.dist;
  }
};

typedef std::priority_queue<PQItemNode,
                       std::vector<PQItemNode>,
                       CompareNode> PriorityQueueNode;

struct PQItemDescriptor {
 public:
  inline explicit PQItemDescriptor(const double d, BinaryDescriptor* bd) :
    dist(d),
    desc(bd) {}

  double dist;
  BinaryDescriptor* desc;
};

class CompareDescriptor {
 public:
  inline bool operator()(const PQItemDescriptor& a, const PQItemDescriptor& b) {
    return a.dist > b.dist;
  }
};

typedef std::priority_queue<PQItemDescriptor,
                       std::vector<PQItemDescriptor>,
                       CompareDescriptor> PriorityQueueDescriptor;

}  // namespace obindex2

#endif  // INCLUDE_OBINDEX2_PRIORITY_QUEUES_H_
