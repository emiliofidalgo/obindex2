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

  BinaryTree::BinaryTree(BinaryDescriptorSetPtr dset,
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
    // Deleting the previous tree, if exists any
    deleteTree();

    // Creating the root node
    root_ = std::make_shared<BinaryTreeNode>();
    nset_.insert(root_);

    // Generating a new set with the descriptor's ids
    BinaryDescriptorSet descs = *dset_;

    buildNode(descs, root_);
  }

  void BinaryTree::buildNode(BinaryDescriptorSet dset, BinaryTreeNodePtr root) {
    // Validate if this should be a leaf node
    if (dset.size() < s_) {
      // We set the previous node as a leaf
      root->setLeaf(true);

      // Adding descriptors as leaf nodes
      for (auto it = dset.begin(); it != dset.end(); it++) {
        BinaryDescriptorPtr d = *it;
        root->addChildDescriptor(d);

        // Storing the reference of the node where the descriptor hangs
        desc_to_node[d] = root;
      }
    } else {
      // This node should be split
      // Randomly selecting the new centers
      std::vector<BinaryDescriptorPtr> new_centers;
      std::vector<BinaryDescriptorSet> assoc_descs(k_);

      for (unsigned i = 0; i < k_; i++) {
        // Selecting a new center
        BinaryDescriptorPtr desc = *std::next(dset.begin(),
                                            rand() % dset.size());
        new_centers.push_back(desc);
        assoc_descs[i].insert(desc);
        dset.erase(desc);
      }

      // Associating the remaining descriptors to the new centers
      for (auto it = dset.begin(); it != dset.end(); it++) {
        BinaryDescriptorPtr d = *it;
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
        BinaryTreeNodePtr node = std::make_shared<BinaryTreeNode>(
                  false,
                  new_centers[i],
                  root);

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
    if (nset_.size() > 0) {
      nset_.clear();

      // Invalidating last reference to root
      root_ = nullptr;
    }
  }

  void BinaryTree::traverseFromRoot(BinaryDescriptorPtr q,
                                    PriorityQueueNodePtr pq,
                                    PriorityQueueDescriptorPtr r) {
    traverseFromNode(q, root_, pq, r);
  }

  void BinaryTree::traverseFromNode(BinaryDescriptorPtr q,
                                    BinaryTreeNodePtr n,
                                    PriorityQueueNodePtr pq,
                                    PriorityQueueDescriptorPtr r) {
    // If its a leaf node, the search ends
    if (n->isLeaf()) {
      // Adding points to R
      std::unordered_set<BinaryDescriptorPtr>* descs =
                                                    n->getChildrenDescriptors();
      for (auto it = (*descs).begin(); it != (*descs).end(); it++) {
        BinaryDescriptorPtr d = *it;
        double dist = obindex2::BinaryDescriptor::distHamming(*q, *d);
        PQItemDescriptor item(dist, d);
        r->push(item);
      }
    } else {
      // Search continues
      std::unordered_set<BinaryTreeNodePtr>* nodes = n->getChildrenNodes();
      int best_node = -1;
      double min_dist = DBL_MAX;

      // Computing distances to nodes
      std::vector<PQItemNode> items;
      unsigned node_id = 0;
      // Computing distances to nodes
      for (auto it = (*nodes).begin(); it != (*nodes).end(); it++) {
        BinaryTreeNodePtr bn = *it;
        double dist = bn->distance(q);
        PQItemNode item(dist, tree_id_, bn);
        items.push_back(item);

        if (dist < min_dist) {
          min_dist = dist;
          best_node = node_id;
        }

        node_id++;
      }

      assert(best_node != -1);

      // Adding remaining nodes to pq
      for (unsigned i = 0; i < items.size(); i++) {
        // Is it the best node?
        if (i == static_cast<unsigned>(best_node)) {
          continue;
        }

        pq->push(items[i]);
      }

      // Traversing the best node
      traverseFromNode(q, items[best_node].node, pq, r);
    }
  }

  BinaryTreeNodePtr BinaryTree::searchFromRoot(BinaryDescriptorPtr q) {
    return searchFromNode(q, root_);
  }

  BinaryTreeNodePtr BinaryTree::searchFromNode(BinaryDescriptorPtr q,
                                               BinaryTreeNodePtr n) {
    // If it's a leaf node, the search ends
    if (n->isLeaf()) {
      // This is the node where this descriptor should be included
      return n;
    } else {
      // Search continues
      std::unordered_set<BinaryTreeNodePtr>* nodes = n->getChildrenNodes();
      int best_node = -1;
      double min_dist = DBL_MAX;

      // Computing distances to nodes
      std::vector<BinaryTreeNodePtr> items;
      // Computing distances to nodes
      for (auto it = (*nodes).begin(); it != (*nodes).end(); it++) {
        BinaryTreeNodePtr bn = *it;
        items.push_back(bn);
        double dist = bn->distance(q);

        if (dist < min_dist) {
          min_dist = dist;
          best_node = static_cast<int>(items.size()) - 1;
        }
      }

      assert(best_node != -1);

      // Searching in the best node
      return searchFromNode(q, items[best_node]);
    }
  }

  void BinaryTree::addDescriptor(BinaryDescriptorPtr q) {
    BinaryTreeNodePtr n = searchFromRoot(q);
    assert(n->isLeaf());
    if (n->childDescriptorSize() + 1 < s_) {
      // There is enough space at this node for this descriptor, so we add it
      n->addChildDescriptor(q);
      // Storing the reference of the node where the descriptor hangs
      desc_to_node[q] = n;
    } else {
      // This node should be split
      n->setLeaf(false);

      // Gathering the current descriptors
      std::unordered_set<BinaryDescriptorPtr>* descs =
                                                    n->getChildrenDescriptors();
      BinaryDescriptorSet set;
      for (auto it = (*descs).begin(); it != (*descs).end(); it++) {
        BinaryDescriptorPtr d = *it;
        set.insert(d);
      }
      set.insert(q);  // Adding the new descritor to the set

      // Rebuilding this node
      buildNode(set, n);
    }
  }

  void BinaryTree::printTree() {
    printNode(root_);
  }

  void BinaryTree::printNode(BinaryTreeNodePtr n) {
    std::cout << "---" << std::endl;
    std::cout << "Node: " << n << std::endl;
    std::cout << (n->isLeaf() ? "Leaf" : "Node") << std::endl;
    std::cout << "Descriptor: " << n->getDescriptor() << std::endl;
    if (n->isLeaf()) {
      std::cout << "Children descriptors: " <<
                             n->childDescriptorSize() << std::endl;
    } else {
      std::cout << "Children nodes: " << n->childNodeSize() << std::endl;
      std::unordered_set<BinaryTreeNodePtr>* nodes = n->getChildrenNodes();
      for (auto it = (*nodes).begin(); it != (*nodes).end(); it++) {
        printNode(*it);
      }
    }
  }

}  // namespace obindex2
