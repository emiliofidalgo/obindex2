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

#include "obindex2/binary_index.h"

namespace obindex2 {

ImageIndex::ImageIndex(const unsigned k,
                       const unsigned s,
                       const unsigned t,
                       const MergePolicy merge_policy) :
    k_(k),
    s_(s),
    t_(t),
    init_(false),
    nimages_(0),
    ndesc_(0),
    merge_policy_(merge_policy) {
      // Validating the corresponding parameters
      assert(k_ > 1);
      assert(k_ < s_);
}

void ImageIndex::addImage(const unsigned image_id,
                          const std::vector<cv::KeyPoint>& kps,
                          const cv::Mat& descs) {
  // Creating the set of BinaryDescriptors
  for (int i = 0; i < descs.rows; i++) {
    // Creating the corresponding descriptor
    cv::Mat desc = descs.row(i);
    BinaryDescriptorPtr d = std::make_shared<BinaryDescriptor>(desc);
    insertDescriptor(d);

    // Creating the inverted index item
    InvIndexItem item;
    item.image_id = image_id;
    item.pt = kps[i].pt;
    item.dist = 0.0;
    item.kp_ind = i;
    inv_index_[d].push_back(item);
  }

  // If the trees are not initialized, we build them
  if (!init_) {
    assert(static_cast<int>(k_) < descs.rows);
    initTrees();
    init_ = true;
  }

  nimages_++;
}

void ImageIndex::addImage(const unsigned image_id,
                const std::vector<cv::KeyPoint>& kps,
                const cv::Mat& descs,
                const std::vector<cv::DMatch>& matches) {
  // --- Adding new features
  // All features
  std::set<int> points;
  for (unsigned feat_ind = 0; feat_ind < kps.size(); feat_ind++) {
    points.insert(feat_ind);
  }

  // Matched features
  std::set<int> matched_points;
  for (unsigned match_ind = 0; match_ind < matches.size(); match_ind++) {
    matched_points.insert(matches[match_ind].queryIdx);
  }

  // Computing the difference
  std::set<int> diff;
  std::set_difference(points.begin(), points.end(),
                      matched_points.begin(), matched_points.end(),
                      std::inserter(diff, diff.end()));

  // Inserting new features into the index.
  for (auto it = diff.begin(); it != diff.end(); it++) {
    int index = *it;
    cv::Mat desc = descs.row(index);
    BinaryDescriptorPtr d = std::make_shared<BinaryDescriptor>(desc);
    insertDescriptor(d);

    // Creating the inverted index item
    InvIndexItem item;
    item.image_id = image_id;
    item.pt = kps[index].pt;
    item.dist = 0.0;
    item.kp_ind = index;
    inv_index_[d].push_back(item);
  }

  // --- Updating the matched descriptors into the index
  for (unsigned match_ind = 0; match_ind < matches.size(); match_ind++) {
    int qindex = matches[match_ind].queryIdx;
    int tindex = matches[match_ind].trainIdx;

    BinaryDescriptorPtr q_d = std::make_shared<BinaryDescriptor>
                                                            (descs.row(qindex));
    BinaryDescriptorPtr t_d = id_to_desc_[tindex];

    // Merge and replace according to the merging policy
    if (merge_policy_ == MERGE_POLICY_NONE) {
    }

    // Creating the inverted index item
    InvIndexItem item;
    item.image_id = image_id;
    item.pt = kps[qindex].pt;
    item.dist = matches[match_ind].distance;
    item.kp_ind = qindex;
    inv_index_[t_d].push_back(item);
  }

  nimages_++;
}

void ImageIndex::searchImages(const cv::Mat& descs,
                              const std::vector<cv::DMatch>& gmatches,
                              std::vector<ImageMatch>* img_matches) {
  // Initializing the resulting structure
  img_matches->resize(nimages_);
  for (unsigned i = 0; i < nimages_; i++) {
    img_matches->at(i).image_id = i;
  }

  // Counting the number of each word in the current document
  std::unordered_map<int, int> nwi_map;
  for (unsigned match_index = 0; match_index < gmatches.size(); match_index++) {
    int train_idx = gmatches[match_index].trainIdx;
    // Updating nwi_map, number of occurrences of a word in an image.
    if (nwi_map.count(train_idx)) {
      nwi_map[train_idx]++;
    } else {
      nwi_map[train_idx] = 1;
    }
  }

  // We process all the matchings again to increase the scores
  for (unsigned match_index = 0; match_index < gmatches.size(); match_index++) {
    int train_idx = gmatches[match_index].trainIdx;
    BinaryDescriptorPtr desc = id_to_desc_[train_idx];

    // Computing the TF term
    double tf = static_cast<double>(nwi_map[train_idx]) / descs.rows;

    // Computing the IDF term
    std::unordered_set<unsigned> nw;
    for (unsigned i = 0; i < inv_index_[desc].size(); i++) {
      nw.insert(inv_index_[desc][i].image_id);
    }
    double idf = log(static_cast<double>(nimages_) / nw.size());

    // Computing the final TF-IDF weighting term
    double tfidf = tf * idf;

    for (unsigned i = 0; i < inv_index_[desc].size(); i++) {
        int im = inv_index_[desc][i].image_id;
        img_matches->at(im).score += tfidf;
    }
  }

  std::sort(img_matches->begin(), img_matches->end());
}

void ImageIndex::initTrees() {
  // Creating the trees
  BinaryDescriptorSetPtr dset_ptr =
                std::make_shared<BinaryDescriptorSet>(dset_);

  for (unsigned i = 0; i < t_; i++) {
    BinaryTreePtr tree_ptr =
              std::make_shared<BinaryTree>(dset_ptr, i, k_, s_);
    trees_.push_back(tree_ptr);
  }
}

void ImageIndex::searchDescriptors(
                              const cv::Mat& descs,
                              std::vector<std::vector<cv::DMatch> >* matches,
                              const unsigned knn,
                              const unsigned checks) {
  matches->clear();
  for (int i = 0; i < descs.rows; i++) {
    // Creating the corresponding descriptor
    cv::Mat desc = descs.row(i);
    BinaryDescriptorPtr d = std::make_shared<BinaryDescriptor>(desc);

    // Searching the descriptor in the index
    std::vector<BinaryDescriptorPtr> neighs;
    std::vector<double> dists;
    searchDescriptor(d, &neighs, &dists, knn, checks);

    // Translating the resulting matches to CV structures
    std::vector<cv::DMatch> des_match;
    for (unsigned j = 0; j < neighs.size(); j++) {
      cv::DMatch match;
      match.queryIdx = i;
      match.trainIdx = static_cast<int>(desc_to_id_[neighs[j]]);
      match.imgIdx = static_cast<int>(inv_index_[neighs[j]][0].image_id);
      match.distance = dists[j];
      des_match.push_back(match);
    }
    matches->push_back(des_match);
  }
}

void ImageIndex::deleteDescriptor(const unsigned desc_id) {
  BinaryDescriptorPtr d = id_to_desc_[desc_id];
  // Clearing the descriptor
  deleteDescriptor(d);
}

void ImageIndex::searchDescriptor(BinaryDescriptorPtr q,
                                  std::vector<BinaryDescriptorPtr>* neigh,
                                  std::vector<double>* distances,
                                  unsigned knn,
                                  unsigned checks) {
  unsigned points_searched = 0;
  PriorityQueueNode pq;
  PriorityQueueDescriptor r;

  // Initializing search structures
  std::vector<PriorityQueueNodePtr> pqs;
  std::vector<PriorityQueueDescriptorPtr> rs;
  for (unsigned i = 0; i < trees_.size(); i++) {
    PriorityQueueNodePtr tpq = std::make_shared<PriorityQueueNode>();
    pqs.push_back(tpq);

    PriorityQueueDescriptorPtr tr = std::make_shared<PriorityQueueDescriptor>();
    rs.push_back(tr);
  }

  // Searching in the trees
  #pragma omp parallel for
  for (unsigned i = 0; i < trees_.size(); i++) {
    trees_[i]->traverseFromRoot(q, pqs[i], rs[i]);
  }

  //  Gathering results from each individual search
  std::unordered_set<BinaryDescriptorPtr> already_added;
  #pragma omp parallel for
  for (unsigned i = 0; i < trees_.size(); i++) {
    // Obtaining priority queue nodes
    while (!pqs[i]->empty()) {
      std::lock_guard<std::mutex> lock(mutex_search_pq_);
      pq.push(pqs[i]->top());
      pqs[i]->pop();
    }

    // Obtaining descriptor nodes
    while (!rs[i]->empty()) {
      PQItemDescriptor r_item = rs[i]->top();
      rs[i]->pop();
      std::lock_guard<std::mutex> lock(mutex_search_r_);
      if (already_added.find(r_item.desc) == already_added.end()) {
        r.push(r_item);
        already_added.insert(r_item.desc);
        points_searched++;
      }
    }
  }

  // Continuing the search if not enough descriptors have been checked
  PriorityQueueNodePtr pq_ptr = std::make_shared<PriorityQueueNode>(pq);
  while (points_searched < checks && !pq.empty()) {
    // Get the closest node to continue the search
    PQItemNode n = pq.top();
    pq.pop();

    // Searching in the node
    PriorityQueueDescriptorPtr tr = std::make_shared<PriorityQueueDescriptor>();
    trees_[n.tree_id]->traverseFromNode(q, n.node, pq_ptr, tr);

    // Obtaining descriptor nodes
    while (!tr->empty()) {
      PQItemDescriptor r_item = tr->top();
      tr->pop();
      if (already_added.find(r_item.desc) == already_added.end()) {
        r.push(r_item);
        already_added.insert(r_item.desc);
        points_searched++;
      }
    }
  }

  // Returning the required number of descriptors descriptors
  neigh->clear();
  distances->clear();
  for (unsigned i = 0; i < knn; i++) {
    if (!r.empty()) {
      PQItemDescriptor d = r.top();
      r.pop();

      neigh->push_back(d.desc);
      distances->push_back(d.dist);
    } else {
      break;
    }
  }
}

void ImageIndex::insertDescriptor(BinaryDescriptorPtr q) {
  dset_.insert(q);
  desc_to_id_[q] = ndesc_;
  id_to_desc_[ndesc_] = q;
  ndesc_++;

  // Indexing the descriptor inside each tree
  if (init_) {
    #pragma omp parallel for
    for (unsigned i = 0; i < trees_.size(); i++) {
      trees_[i]->addDescriptor(q);
    }
  }
}

void ImageIndex::deleteDescriptor(BinaryDescriptorPtr q) {
  // Deleting the descriptor from each tree
  if (init_) {
    #pragma omp parallel for
    for (unsigned i = 0; i < trees_.size(); i++) {
      trees_[i]->deleteDescriptor(q);
    }
  }

  dset_.erase(q);
  unsigned desc_id = desc_to_id_[q];
  desc_to_id_.erase(q);
  id_to_desc_.erase(desc_id);
}

}  // namespace obindex2
