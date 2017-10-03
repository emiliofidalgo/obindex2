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

#include "obindex2/binary_tree.h"

namespace obindex2 {

  BinaryTree::BinaryTree(BinaryDescriptorSet* dset,
                         const unsigned tree_id,
                         const unsigned k,
                         const unsigned s) :
    dset_(dset),
    tree_id_(tree_id),
    root_(nullptr),
    k_(k),
    s_(s) {
      srand(time(NULL));
      buildTree();
  }

  BinaryTree::~BinaryTree() {
    deleteTree();
  }

  void BinaryTree::buildTree() {
    assert(k_ < dset_.size());

    // Deleting the previous tree, if exists any
    deleteTree();

    // Creating the root node
    root_ = new BinaryTreeNode();
    nset_.insert(root_);

    // Generating a new set with the descriptor's ids
    BinaryDescriptorSet descs = *dset_;

    buildNode(descs, root_);
  }

  void BinaryTree::buildNode(BinaryDescriptorSet dset, BinaryTreeNode* root) {
    // Validate if this should be a leaf node
    if (dset.size() < s_) {
      // We set the previous node as a leaf
      root->setLeaf(true);

      // Adding descriptors as leaf nodes
      for (auto it = dset.begin(); it != dset.end(); it++) {
        root->addChildDescriptor(*it);
      }

    } else {
      // This node should be split
      // Randomly selecting the new centers
      std::vector<BinaryDescriptor*> new_centers;
      std::vector<BinaryDescriptorSet> assoc_descs(k_);

      for (unsigned i = 0; i < k_; i++) {
        // Selecting a new center
        BinaryDescriptor* desc = *std::next(dset.begin(),
                                            rand() % dset.size());
        new_centers.push_back(desc);
        assoc_descs[i].insert(desc);
        dset.erase(desc);
      }

      // Associating the remaining descriptors to the new centers
      for (auto it = dset.begin(); it != dset.end(); it++) {
        BinaryDescriptor* d = *it;
        int best_center = -1;
        double min_dist = DBL_MAX;
        for (unsigned i = 0; i < k_; i++) {
          double dist =
            obindex2::BinaryDescriptor::distHamming(*d,
                                                    *(new_centers[i]));

          if (dist < min_dist) {
            min_dist = dist;
            best_center = i;
          }
        }

        assert(best_center != -1);
        assoc_descs[best_center].insert(d);
      }
      dset.clear();

      // Creating a new tree node for each new cluster
      for (unsigned i = 0; i < k_; i++) {
        BinaryTreeNode* node = new BinaryTreeNode(false, new_centers[i]);

        // Linking this node with its root
        root->addChildNode(node);

        // Storing the reference to this node
        nset_.insert(node);

        // Recursively apply the algorithm
        buildNode(assoc_descs[i], node);
      }
    }
  }

  void BinaryTree::deleteTree() {
    // if (nset_.size() > 0) {
    //   // Deleting tree nodes
    //   for (auto it = nset_.begin(); it != nset_.end(); it++) {
    //     BinaryTreeNode* node = *it;
    //     delete node;
    //   }

    //   // Invalidating pointers
    //   root_ = nullptr;
    // }
  }

  void BinaryTree::traverseFromRoot(BinaryDescriptor* q,
                                    PriorityQueueNode* pq,
                                    PriorityQueueDescriptor* r) {
    traverseFromNode(q, root_, pq, r);
  }

  void BinaryTree::traverseFromNode(BinaryDescriptor* q,
                                    BinaryTreeNode* n,
                                    PriorityQueueNode* pq,
                                    PriorityQueueDescriptor* r) {
    // If its a leaf node, the search ends
    if (n->isLeaf()) {
      // Adding points to R
      std::vector<BinaryDescriptor*>* descs = n->getChildrenDescriptors();
      for (unsigned i = 0; i < descs->size(); i++) {
        BinaryDescriptor* d = (*descs)[i];
        double dist = obindex2::BinaryDescriptor::distHamming(*q, *d);
        PQItemDescriptor item(dist, d);
        r->push(item);
      }
    } else {
      // Search continues
      std::vector<BinaryTreeNode*>* nodes = n->getChildrenNodes();
      int best_node = -1;
      double min_dist = DBL_MAX;

      // Computing distances to nodes
      std::vector<double> distances(nodes->size());
      for (unsigned i = 0; i < nodes->size(); i++) {
        BinaryTreeNode* bn = (*nodes)[i];
        double dist = bn->distance(q);
        distances[i] = dist;

        if (dist < min_dist) {
          min_dist = dist;
          best_node = i;
        }
      }

      assert(best_node != -1);

      // Adding remaining nodes to pq
      for (unsigned i = 0; i < nodes->size(); i++) {
        BinaryTreeNode* bn = (*nodes)[i];
        // It is the best node?
        if (i == static_cast<unsigned>(best_node)) {
          continue;
        }

        PQItemNode item(distances[i], tree_id_, bn);
        pq->push(item);
      }

      // Traversing the best node
      traverseFromNode(q, (*nodes)[best_node], pq, r);
    }
  }

  void BinaryTree::printTree() {
    printNode(root_);
  }

  void BinaryTree::printNode(BinaryTreeNode* n) {
    std::cout << "---" << std::endl;
    std::cout << "Node: " << n << std::endl;
    std::cout << (n->isLeaf() ? "Leaf" : "Node") << std::endl;
    std::cout << "Descriptor: " << n->getDescriptor() << std::endl;
    if (n->isLeaf()) {
      std::cout << "Children descriptors: " <<
                             n->childDescriptorSize() << std::endl;
    } else {
      std::cout << "Children nodes: " << n->childNodeSize() << std::endl;
      std::vector<BinaryTreeNode*>* nodes = n->getChildrenNodes();
      for (unsigned i = 0; i < nodes->size(); i++) {
        printNode((*nodes)[i]);
      }
    }
  }

}  // namespace obindex2
