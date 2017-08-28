#include <jni.h>
#include "ar_jni.h"
#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/feature.h>
#include <svo/map.h>
#include <svo/fast.h>
#include <svo/frame.h>
#include <vikit/vision.h>
#include <vector>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Geometry>

#include <android/log.h>

using namespace Eigen;
using namespace std;

extern int cameraWidth;
extern int cameraHeight;
extern svo::FrameHandlerMono *frameHandler;

static auto detector = cv::BRISK::create(
  20, // thresh
  4, // octaves
  1.0f // pattern scale
);

static auto extractor = cv::xfeatures2d::FREAK::create(
  true, // orientationNormalized
  true, // scaleNormalized
  22.0f, // patternScale,
  4 // nOctaves,
);

static std::vector<cv::KeyPoint> fingerKeypoints;
static cv::Mat fingerDescriptors;
static cv::Mat fingerTransformation;

// svo::FramePtr fingerFrame;
// static std::vector<svo::Feature *> fingerFeatures;
JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1setMarker
    (JNIEnv *env, jclass, jint sx, jint sy, jint ex, jint ey) {
  auto fingerFrame = frameHandler->lastFrame();
  if(!fingerFrame.get()) return;

  std::vector<cv::KeyPoint> keypoints;
  detector->detect(fingerFrame->img_pyr_[1], keypoints);

  fingerKeypoints.clear();
  for(auto &kp: keypoints) {
    if(kp.pt.x * 2 > sx && kp.pt.x * 2 < ex &&
        kp.pt.y * 2 > sy && kp.pt.y * 2 < ey) {
      fingerKeypoints.push_back(kp);
    }
  }

  extractor->compute(fingerFrame->img_pyr_[1], fingerKeypoints, fingerDescriptors);

  __android_log_print(ANDROID_LOG_INFO, "Gesture", "# features: %d", fingerDescriptors.rows);
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1processFrame
  (JNIEnv *env, jclass, jfloatArray debugImage) {

  auto fingerFrame = frameHandler->lastFrame();
  if(!fingerFrame.get()) return;

// Use SVO-based feature detector
//  jfloat *debugData = env->GetFloatArrayElements(debugImage, 0);
//  for(int y = 0; y < cameraHeight; ++y) {
//    for(int x = 0; x < cameraWidth; ++x) {
//      debugData[y * cameraWidth + x] = fingerFrame->img_pyr_[0].at<unsigned char>(y, x) / 512.0f;
//    }
//  }
//
//  for(int L = 0; L < fingerFrame->img_pyr_.size(); ++L) {
//    const int scale = 1 << L;
//    vector<fast::fast_xy> fast_corners;
//#if __SSE2__
//      fast::fast_corner_detect_10_sse2(
//          (fast::fast_byte*) fingerFrame->img_pyr_[L].data, fingerFrame->img_pyr_[L].cols,
//          fingerFrame->img_pyr_[L].rows, fingerFrame->img_pyr_[L].cols, 20, fast_corners);
//#else
//      fast::fast_corner_detect_10(
//          (fast::fast_byte*) fingerFrame->img_pyr_[L].data, fingerFrame->img_pyr_[L].cols,
//          fingerFrame->img_pyr_[L].rows, fingerFrame->img_pyr_[L].cols, 20, fast_corners);
//#endif
//    vector<int> scores, nm_corners;
//    fast::fast_corner_score_10((fast::fast_byte*) fingerFrame->img_pyr_[L].data,
//        fingerFrame->img_pyr_[L].cols, fast_corners, 20, scores);
//    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);
//
//    for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
//    {
//      fast::fast_xy& xy = fast_corners.at(*it);
//      const float score = vk::shiTomasiScore(fingerFrame->img_pyr_[L], xy.x, xy.y);
//      if(score > 20.0) {
//        const int x = xy.x * scale;
//        const int y = xy.y * scale;
//
//        debugData[y * cameraWidth + x - 1] = 1.0;
//        debugData[y * cameraWidth + x] = 1.0;
//        debugData[y * cameraWidth + x + 1] = 1.0;
//      }
//    }
//  }
//  env->ReleaseFloatArrayElements(debugImage, debugData, 0);

  std::vector<cv::KeyPoint> keypoints;
  detector->detect(fingerFrame->img_pyr_[1], keypoints);

  cv::Mat descriptors;
  extractor->compute(fingerFrame->img_pyr_[1], keypoints, descriptors);

  jfloat *debugData = env->GetFloatArrayElements(debugImage, 0);
  for(int y = 0; y < cameraHeight; ++y) {
    for(int x = 0; x < cameraWidth; ++x) {
      debugData[y * cameraWidth + x] = fingerFrame->img_pyr_[0].at<unsigned char>(y, x) / 512.0f;
    }

    debugData[y * cameraWidth + y - 1] = 1.0;
    debugData[y * cameraWidth + y] = 1.0;
    debugData[y * cameraWidth + y + 1] = 1.0;
  }

  for(auto &kp: keypoints) {
    debugData[(int)kp.pt.y * 2 * cameraWidth + (int)kp.pt.x * 2 - 1] = 1.0;
    debugData[(int)kp.pt.y * 2 * cameraWidth + (int)kp.pt.x * 2 ] = 1.0;
    debugData[(int)kp.pt.y * 2 * cameraWidth + (int)kp.pt.x * 2  + 1] = 1.0;
  }

  __android_log_print(ANDROID_LOG_INFO, "Gesture", "# known features: %d", fingerDescriptors.rows);
  if(fingerDescriptors.rows) {
    // TODO: Potentially use FLANN + correct indexing only after finger features acquired
    // cv::FlannBasedMatcher matcher;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> matches;

    // TODO: Test knn matching here
    matcher.knnMatch(fingerDescriptors, descriptors, matches, 3);

    double max_dist = 0;
    double min_dist = 100;

    for(auto &ms: matches) {
      for(auto &m: ms) {
        const double dist = m.distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
      }
    }

    __android_log_print(ANDROID_LOG_INFO, "Gesture", "Max / Min: %f / %f", max_dist, min_dist);

    std::vector<cv::Point2f> finger;
    std::vector<cv::Point2f> scene;

    for(auto &ms: matches) {
      for(auto &m: ms) {
        if(m.distance > 60.0) continue;

        finger.push_back(fingerKeypoints[m.queryIdx].pt);
        scene.push_back(keypoints[m.trainIdx].pt);

        __android_log_print(ANDROID_LOG_INFO, "Gesture", "Match: %f,%f <-> %f,%f",
            finger.back().x, finger.back().y,
            scene.back().x, scene.back().y);
      }
    }

    __android_log_print(ANDROID_LOG_INFO, "Gesture", "Features identified: %d", scene.size());

    fingerTransformation = cv::Mat();
    if(scene.size() >= 4) {
      cv::Mat H = cv::findHomography(finger, scene, CV_RANSAC);

      __android_log_print(ANDROID_LOG_INFO, "Gesture", "H dim %dx%d", H.rows, H.cols);
      if(H.rows == 3 && H.cols == 3) {
        fingerTransformation = H;

        cv::Mat pos = (cv::Mat_<double>(3, 1) << cameraWidth / 4, cameraHeight / 4, 1);
        pos = H * pos;

        const double x = pos.at<double>(0);
        const double y = pos.at<double>(1);
        const double w = pos.at<double>(2);
        __android_log_print(ANDROID_LOG_INFO, "Gesture", "Finger X,Y: %lf,%lf", x / w, y / w);
      }
    }
  }

  env->ReleaseFloatArrayElements(debugImage, debugData, 0);
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1getTransformationRelative
  (JNIEnv *env, jclass, jlong time_nano, jfloatArray transformation) {

  if(env->IsSameObject(transformation, nullptr)) {
    env->ThrowNew(env->FindClass("java/lang/NullPointerException"),
        "null passed for output buffer");
    return;
  }

  jfloat *transformationData = env->GetFloatArrayElements(transformation, 0);

  if(fingerTransformation.rows == 3 && fingerTransformation.cols == 3) {
    for(int y = 0; y < 3; ++y) {
      for(int x = 0; x < 3; ++x) {
        transformationData[y * 3 + x] = fingerTransformation.at<double>(y, x);
      }
    }
  } else {
    for(int y = 0; y < 3; ++y) {
      for(int x = 0; x < 3; ++x) {
        transformationData[y * 3 + x] = 0;
      }
    }
  }

  env->ReleaseFloatArrayElements(transformation, transformationData, 0);
}
