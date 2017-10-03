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

#ifndef INCLUDE_OBINDEX2_BINARY_TREE_H_
#define INCLUDE_OBINDEX2_BINARY_TREE_H_

#include <stdlib.h>
#include <time.h>

#include <limits>
#include <unordered_set>

#include "obindex2/binary_descriptor.h"
#include "obindex2/priority_queues.h"

namespace obindex2 {

class BinaryTree {
 public:
  // Constructors
  explicit BinaryTree(BinaryDescriptorSetPtr dset,
                      const unsigned tree_id = 0,
                      const unsigned k = 16,
                      const unsigned s = 150);
  virtual ~BinaryTree();

  // Methods
  void buildTree();
  void deleteTree();
  void traverseFromRoot(BinaryDescriptorPtr q,
                        PriorityQueueNodePtr pq,
                        PriorityQueueDescriptorPtr r);
  void traverseFromNode(BinaryDescriptorPtr q,
                        BinaryTreeNodePtr n,
                        PriorityQueueNodePtr pq,
                        PriorityQueueDescriptorPtr r);
  void printTree();

 private:
  BinaryDescriptorSetPtr dset_;
  unsigned tree_id_;
  BinaryTreeNodePtr root_;
  unsigned k_;
  unsigned s_;
  NodeSet nset_;

  void buildNode(BinaryDescriptorSet d, BinaryTreeNodePtr root);
  void printNode(BinaryTreeNodePtr n);
};

}  // namespace obindex2

#endif  // INCLUDE_OBINDEX2_BINARY_TREE_H_
