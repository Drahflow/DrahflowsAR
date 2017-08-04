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

#include <Eigen/Geometry>

#include <android/log.h>

using namespace Eigen;
using namespace std;

extern int cameraWidth;
extern int cameraHeight;
extern svo::FrameHandlerMono *frameHandler;

svo::FramePtr fingerFrame;
static std::vector<svo::Feature *> fingerFeatures;

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1setMarker
    (JNIEnv *env, jclass, jint sx, jint sy, jint ex, jint ey) {
  for(auto &f: fingerFeatures) delete f;
  fingerFeatures.clear();

  const double detection_threshold = svo::Config::getInstance().triangMinCornerScore();

  fingerFrame = frameHandler->lastFrame();
  if(!fingerFrame.get()) return;

  for(int L = 0; L < fingerFrame->img_pyr_.size(); ++L) {
    const int scale = 1 << L;
    vector<fast::fast_xy> fast_corners;
#if __SSE2__
      fast::fast_corner_detect_10_sse2(
          (fast::fast_byte*) fingerFrame->img_pyr_[L].data, fingerFrame->img_pyr_[L].cols,
          fingerFrame->img_pyr_[L].rows, fingerFrame->img_pyr_[L].cols, 20, fast_corners);
#else
      fast::fast_corner_detect_10(
          (fast::fast_byte*) fingerFrame->img_pyr_[L].data, fingerFrame->img_pyr_[L].cols,
          fingerFrame->img_pyr_[L].rows, fingerFrame->img_pyr_[L].cols, 20, fast_corners);
#endif
    vector<int> scores, nm_corners;
    fast::fast_corner_score_10((fast::fast_byte*) fingerFrame->img_pyr_[L].data,
        fingerFrame->img_pyr_[L].cols, fast_corners, 20, scores);
    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

    for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
    {
      fast::fast_xy& xy = fast_corners.at(*it);
      const float score = vk::shiTomasiScore(fingerFrame->img_pyr_[L], xy.x, xy.y);
      // FIXME: 
      __android_log_print(ANDROID_LOG_INFO, "Gesture", "score: %f (of %f)", score, detection_threshold);
      if(score > detection_threshold) {
        const int x = xy.x * scale;
        const int y = xy.y * scale;
        if(x >= sx && x < ex && y > sy && y < ey) {
          __android_log_print(ANDROID_LOG_INFO, "Gesture", "Feature: %d,%d", xy.x * scale, xy.y * scale);

          fingerFeatures.push_back(new svo::Feature(fingerFrame.get(),
                Vector2d(xy.x * scale, xy.y * scale), L));
        }
      }
    }
  }

  __android_log_print(ANDROID_LOG_INFO, "Gesture", "# features: %d", fingerFeatures.size());
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1processFrame
  (JNIEnv *env, jclass) {
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1getTransformationRelative
  (JNIEnv *env, jclass, jlong time_nano, jfloatArray transformation) {
}
