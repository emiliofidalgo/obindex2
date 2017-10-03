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

#include <opencv2/xfeatures2d.hpp>

#include "obindex2/binary_tree.h"

int main() {
  // Creating feature detector and descriptor
  cv::Ptr<cv::FastFeatureDetector> det =
          cv::FastFeatureDetector::create();
  cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> des =
          cv::xfeatures2d::BriefDescriptorExtractor::create();

  // Loading the test image
  cv::Mat img = cv::imread("image.jpg");

  // Computing keypoints and descriptors
  std::vector<cv::KeyPoint> kps;
  det->detect(img, kps);

  cv::Mat descs;
  des->compute(img, kps, descs);

  // Creating a set of descriptors
  obindex2::BinaryDescriptorSet set;
  for (int i = 0; i < descs.rows; i++) {
    cv::Mat desc = descs.row(i);
    obindex2::BinaryDescriptorPtr d =
      std::make_shared<obindex2::BinaryDescriptor>(desc);
    set.insert(d);
  }

  obindex2::BinaryTree tree1(std::make_shared<
                             obindex2::BinaryDescriptorSet>(set));

  tree1.deleteTree();
  tree1.buildTree();

  // Searching in the tree
  for (auto it = set.begin(); it != set.end(); it++) {
    obindex2::BinaryDescriptorPtr q = *it;
    obindex2::PriorityQueueNodePtr pq = std::make_shared<
                                          obindex2::PriorityQueueNode>();
    obindex2::PriorityQueueDescriptorPtr r = std::make_shared<
                                          obindex2::PriorityQueueDescriptor>();

    tree1.traverseFromRoot(q, pq, r);

    std::cout << "---" << std::endl;

    while (!r->empty()) {
     std::cout << r->top().dist << " " <<
          r->top().desc << " vs " << q << std::endl;
     r->pop();
    }
  }

  return 0;  // Correct test
}
