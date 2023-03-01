#include "../include/feature_matcher.h"

FeatureMatcher::FeatureMatcher() {}

void FeatureMatcher::init(std::string ref_img_path, int knn_k)
{
  mDetector = cv::ORB::create();
  mExtractor = cv::ORB::create();

  // parameters taken from
  // https://stackoverflow.com/questions/43830849/opencv-use-flann-with-orb-descriptors-to-match-features
  // there could be better values for these
  mMatcher =
      cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));

  // get reference image data
  getReferenceFrame(ref_img_path);
  mKNN_K = knn_k;
}

void FeatureMatcher::getReferenceFrame(std::string img_path)
{
  // Idk if any of these throw if anything goes wrong. Should probably check
  // myself.
  cv::Mat ref_img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
  mDetector->detect(ref_img, mReferenceFrameData.kp);
  mExtractor->compute(ref_img, mReferenceFrameData.kp, mReferenceFrameData.des);
}

void FeatureMatcher::match(cv::Mat &des, int k)
{
  mDummyMatches.clear(); // may not be necessary
  mMatcher.knnMatch(des, mReferenceFrameData.des, mDummyMatches, k);
  // mMatcher.match(des, mReferenceFrameData.des, matches);
}

void FeatureMatcher::filter()
{
  // Apply ratio test.
  mDummyGoodMatches.clear();
  constexpr float ratio_thresh = 0.7;
  for (size_t i = 0; i < mDummyMatches.size(); i++)
  {
    // idk sometimes there are no matches??
    if (mDummyMatches[i].size() != 2)
      continue;

    // std::cout << "mDummyMatches.size(): " << mDummyMatches.size() <<
    // std::endl; std::cout << "mDummyMatches[" << i << "].size(): " <<
    // mDummyMatches[i].size() << std::endl << std::endl; std::cout <<
    // "mDummyMatches[" << i << "][0].distance: " <<
    // mDummyMatches[i][0].distance << std::endl; std::cout <<
    // "mDummyMatches[" << i << "][1].distance: " <<
    // mDummyMatches[i][1].distance << std::endl;

    if (mDummyMatches[i][0].distance <
        ratio_thresh * mDummyMatches[i][1].distance)
    {
      // std::cout << "in\n";
      mDummyGoodMatches.push_back(mDummyMatches[i][0]);
    }
    // std::cout << "out\n";
  }
}

// for testing
cv::Mat FeatureMatcher::drawMatches(cv::Mat img1, cv::Mat img2)
{
  // img1 -> ref_img, img2 -> frame
  cv::Mat out_img;
  cv::drawMatches(img2, mDummyKp, img1, mReferenceFrameData.kp,
                  mDummyGoodMatches, out_img);
  return out_img;
}

// for testing
cv::Mat FeatureMatcher::drawGoodMatches(cv::Mat img1, cv::Mat img2)
{
  // img1 -> ref_img, img2 -> frame
  cv::Mat out_img;
  cv::drawMatches(img2, mDummyKp, img1, mReferenceFrameData.kp, mDummyMatches,
                  out_img);
  return out_img;
}

void FeatureMatcher::drawDetections(cv::Mat &frame)
{
  // TODO:
}

bool FeatureMatcher::findObject(cv::Mat &frame)
{
  // I can use dummy member descriptors and keypoints here
  // so that we don't need to allocate each time.
  // I doubt compiler handles that.
  mDummyMatches.clear();
  mDummyGoodMatches.clear();

  mDetector->detect(frame, mDummyKp);
  mExtractor->compute(frame, mDummyKp, mDummyDes);

  match(mDummyDes, KNN_K);
  filter();

  return true;
}

std::vector<double> FeatureMatcher::getMatchCounts()
{
  std::vector<double> counts;
  counts.push_back(mDummyMatches.size());
  counts.push_back(mDummyGoodMatches.size());
  return counts;
}

// may delete later
void FeatureMatcher::setObj(std::shared_ptr<MasterObject> m) { mObj = m; }
