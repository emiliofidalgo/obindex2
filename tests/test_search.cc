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

#include <cstdio>
#include <iostream>

#include <boost/filesystem.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "obindex2/binary_index.h"

void getFilenames(const std::string& directory,
                  std::vector<std::string>* filenames) {
    using namespace boost::filesystem;

    filenames->clear();
    path dir(directory);

    // Retrieving, sorting and filtering filenames.
    std::vector<path> entries;
    copy(directory_iterator(dir), directory_iterator(), back_inserter(entries));
    sort(entries.begin(), entries.end());
    for (auto it = entries.begin(); it != entries.end(); it++) {
        std::string ext = it->extension().c_str();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".png" || ext == ".jpg" ||
            ext == ".ppm" || ext == ".jpeg") {
            filenames->push_back(it->string());
        }
    }
}

int main() {
  // Creating feature detector and descriptor
  cv::Ptr<cv::FastFeatureDetector> det =
          cv::FastFeatureDetector::create();
  cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> des =
          cv::xfeatures2d::BriefDescriptorExtractor::create();

  // Loading image filenames
  std::vector<std::string> filenames;
  getFilenames(".", &filenames);
  unsigned nimages = filenames.size();

  // Creating a new index of images
  obindex2::ImageIndex index(16, 150, 4);

  // Adding image 0
  // Detecting and describing keypoints
  std::vector<cv::KeyPoint> kps0;
  cv::Mat dscs0;
  cv::Mat image0 = cv::imread(filenames[0]);
  det->detect(image0, kps0);
  cv::KeyPointsFilter::retainBest(kps0, 1000);
  des->compute(image0, kps0, dscs0);

  // Adding the image to the index.
  index.addImage(0, kps0, dscs0);

  for (unsigned i = 1; i < nimages; i++) {
    std::cout << "---" << std::endl;
    // Processing image i
    std::cout << "Processing image " << i << std::endl;
    // Loading and describing the image
    cv::Mat img = cv::imread(filenames[i]);
    std::vector<cv::KeyPoint> kps;
    cv::Mat dscs;
    det->detect(img, kps);
    cv::KeyPointsFilter::retainBest(kps, 1000);
    des->compute(img, kps, dscs);

    // Matching the descriptors
    std::vector<std::vector<cv::DMatch> > matches_feats;

    // Searching the query descriptors against the features
    index.searchDescriptors(dscs, &matches_feats, 2, 64);

    // Filtering matches according to the ratio test
    std::vector<cv::DMatch> matches;
    for (unsigned m = 0; m < matches_feats.size(); m++) {
      if (matches_feats[m][0].distance < matches_feats[m][1].distance * 0.8) {
        matches.push_back(matches_feats[m][0]);
      }
    }

    std::vector<obindex2::ImageMatch> image_matches;

    // We look for similar images according to the good matches found
    index.searchImages(dscs, matches, &image_matches);

    // Showing results
    for (int j = 0; j < std::min(5, static_cast<int>(image_matches.size()));
                                                                          j++) {
      std::cout << "Cand: " << image_matches[j].image_id <<  ", " <<
                   "Score: " << image_matches[j].score << std::endl;
    }

    std::cout << "Total features found in the image: " <<
                                          kps.size() << std::endl;
    std::cout << "Total matches found against the index: " <<
                                          matches.size() << std::endl;
    std::cout << "Total index size BEFORE UPDATE: " <<
                                          index.numDescriptors() << std::endl;
    // Updating the index
    // Matched descriptors are used to update the index and the remaining ones
    // are added as new visual words
    index.addImage(i, kps, dscs, matches);
    std::cout << "Total index size AFTER UPDATE: " <<
                                          index.numDescriptors() << std::endl;
  }

  return 0;  // Correct test
}
