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

struct HandDescription {
  float w, h;
  vector<float> fingerSignature;
  float fingerSignatureMean, fingerSignatureSquare;

  vector<float> crossSignature;
  float crossSignatureMean, crossSignatureSquare;

  HandDescription() {
    w = 0;
  }
};

const bool CROSSCORR_POSITIVE_ONLY = true;
const float HAND_TRACKER_POINTS = 15;
const float HAND_TRACKER_POINTS_CROSS = 20;
const int HAND_TRACKER_WINDOW_MIN = 30;
const int HAND_TRACKER_WINDOW_MAX = 200;

struct ThresholdConfiguration {
  public:
    bool searchChecks;
    float contrastThreshold;
    float crossContrastThreshold;
    float brightnessThreshold;
    float fingersThreshold;
    float crossThreshold;
    float trackingQualityThreshold;
};

const ThresholdConfiguration SEARCH_THRESHOLDS = {
  searchChecks: true,
  contrastThreshold: 0.6,
  crossContrastThreshold: 0.4,
  brightnessThreshold: 75,
  fingersThreshold: 0.7,
  crossThreshold: 0.5,
  trackingQualityThreshold: 0,
};

const ThresholdConfiguration TRACKING_THRESHOLDS = {
  searchChecks: false,
  contrastThreshold: 0.6,
  crossContrastThreshold: 0.6,
  brightnessThreshold: 50,
  fingersThreshold: 0.7,
  crossThreshold: 0.6,
  trackingQualityThreshold: 0,
};

const ThresholdConfiguration MEASURE_THRESHOLDS = {
  searchChecks: false,
  contrastThreshold: 0.1,
  crossContrastThreshold: 0.1,
  brightnessThreshold: 125,
  fingersThreshold: 0.45,
  crossThreshold: 0.25,
  trackingQualityThreshold: 0.4,
};

float interpolate(cv::Mat &img, float x, float y) {
  return img.at<unsigned char>(y + 0.5, x + 0.5);
}

template<typename I> typename iterator_traits<I>::value_type mean(
    const I &is, const I &ie) {
  typename iterator_traits<I>::value_type sum = 0;
  int count = 0;

  for(auto i = is; i != ie; ++i) {
    sum += *i;
    count++;
  }

  return sum / count;
}

template<typename I, typename IT, typename J, typename JT>
typename iterator_traits<I>::value_type meanDotProduct(
    const I &is, const I &ie, const IT &im, const J &js, const J &, const JT &jm) {
  typename iterator_traits<I>::value_type p = 0;

  auto i = is;
  auto j = js;

  while(i != ie) {
    p += (*i - im) * (*j - jm);
    ++i; ++j;
  }

  return p;
}

template<typename I, typename IT>
typename iterator_traits<I>::value_type meanDotSquare(
    const I &is, const I &ie, const IT &im) {
  typename iterator_traits<I>::value_type p = 0;

  auto i = is;

  while(i != ie) {
    p += (*i - im) * (*i - im);
    ++i;
  }

  return p;
}

template<typename I, typename J, typename JT>
typename I::value_type meanDotProductLeftConst(
    const I &is, const I &ie, const J &js, const J &, const JT &jm) {
  typename I::value_type p = 0;

  auto i = is;
  auto j = js;

  while(i != ie) {
    p += *i * (*j - jm);
    ++i; ++j;
  }

  return p;
}

template<typename I, typename J>
typename I::value_type meanDotProductLeftConstRightConst(
    const I &is, const I &ie, const J &js, const J &) {
  typename I::value_type p = 0;

  auto i = is;
  auto j = js;

  while(i != ie) {
    p += *i * *j;
    ++i; ++j;
  }

  return p;
}

template<typename T>
struct CorrelationCoefficient {
  T corr;
  T beta, gamma;
};

// template<typename I, typename J>
// CorrelationCoefficient<typename I::value_type> correlationCoefficient(
//     const I &is, const I &ie, const J &js, const J &je,
//     const float &contrastThreshold, const float &brightnessThreshold) {
//   struct CorrelationCoefficient<typename I::value_type> result;
//
//   const typename I::value_type xm = mean(is, ie);
//   const typename iterator_traits<J>::value_type ym = mean(js, je);
//
//   const typename I::value_type xx = meanDotSquare(is, ie, xm);
//   const typename I::value_type xy = meanDotProduct(is, ie, xm, js, je, ym);
//
//   float beta = xy / xx;
//   float gamma = ym - beta * xm;
//
//   result.beta = beta;
//   result.gamma = gamma;
//
//   if(CROSSCORR_POSITIVE_ONLY && beta < 0) {
//     result.corr = 0;
//     return result;
//   }
//
//   if(fabs(beta) < contrastThreshold || 1 / contrastThreshold < fabs(beta)) {
//     result.corr = 0;
//     return result;
//   }
//
//   if(fabs(gamma) > brightnessThreshold) {
//     result.corr = 0;
//     return result;
//   }
//
//   const typename iterator_traits<J>::value_type yy = meanDotSquare(js, je, ym);
//   result.corr = xy / sqrt(xx * yy);
//
//   return result;
// }

template<typename I, typename J>
CorrelationCoefficient<typename I::value_type> correlationCoefficientLeftConst(
    const I &is, const I &ie, const typename I::value_type im, const typename I::value_type i2,
    const J &js, const J &je,
    const float &contrastThreshold, const float &brightnessThreshold) {
  struct CorrelationCoefficient<typename I::value_type> result;

  const typename iterator_traits<I>::value_type xm = im;
  const typename iterator_traits<J>::value_type ym = mean(js, je);

  const typename I::value_type xx = i2;
  const typename I::value_type xy = meanDotProductLeftConst(is, ie, js, je, ym);

  float beta = xy / xx;
  float gamma = ym - beta * xm;

  result.beta = beta;
  result.gamma = gamma;

  if(CROSSCORR_POSITIVE_ONLY && beta < 0) {
    result.corr = 0;
    return result;
  }

  if(fabs(beta) < contrastThreshold || 1 / contrastThreshold < fabs(beta)) {
    result.corr = 0;
    return result;
  }

  if(fabs(gamma) > brightnessThreshold) {
    result.corr = 0;
    return result;
  }

  const typename iterator_traits<J>::value_type yy = meanDotSquare(js, je, ym);
  result.corr = xy / sqrt(xx * yy);

  return result;
}

template<typename I, typename J>
CorrelationCoefficient<typename I::value_type> correlationCoefficientLeftConstRightConst(
    const I &is, const I &ie, const typename I::value_type im, const typename I::value_type i2,
    const J &js, const J &je, const typename J::value_type jm, const typename J::value_type j2,
    const float &contrastThreshold, const float &brightnessThreshold) {
  struct CorrelationCoefficient<typename I::value_type> result;

  const typename iterator_traits<I>::value_type xm = im;
  const typename iterator_traits<J>::value_type ym = jm;

  const typename I::value_type xx = i2;
  const typename I::value_type xy = meanDotProductLeftConstRightConst(is, ie, js, je);

  float beta = xy / xx;
  float gamma = ym - beta * xm;

  result.beta = beta;
  result.gamma = gamma;

  if(CROSSCORR_POSITIVE_ONLY && beta < 0) {
    result.corr = 0;
    return result;
  }

  if(fabs(beta) < contrastThreshold || 1 / contrastThreshold < fabs(beta)) {
    result.corr = 0;
    return result;
  }

  if(fabs(gamma) > brightnessThreshold) {
    result.corr = 0;
    return result;
  }

  const typename iterator_traits<J>::value_type yy = j2;
  result.corr = xy / sqrt(xx * yy);

  return result;
}

void sampleFinger(vector<float> &measured, const HandDescription &obj,
    cv::Mat &img, int x, int y, float scale) {
  float xi = x - (obj.w / 2 * scale);
  const float dx = obj.w * scale / HAND_TRACKER_POINTS;
  if(y < 0 || y > img.rows) {
    for(int i = 0; i < HAND_TRACKER_POINTS; ++i, xi += dx) {
      measured.push_back(-1e6);
    }
    return;
  }

  for(int i = 0; i < HAND_TRACKER_POINTS; ++i, xi += dx) {
    if(xi < 0 || xi >= img.cols) {
      measured.push_back(-1e6);
    } else {
      measured.push_back(interpolate(img, xi, y));
    }
  }
}

void sampleCross(vector<float> &measured, const HandDescription &obj,
    cv::Mat &img, int x, int y, float scale) {
  float xi = x - (obj.w / 2 * scale);
  for(int i = 0; i < HAND_TRACKER_POINTS; ++i, xi += obj.w * scale / HAND_TRACKER_POINTS_CROSS) {
    float yi = y - (obj.h / 2 * scale);
    for(int j = 0; j < HAND_TRACKER_POINTS; ++j, yi += obj.h * scale / HAND_TRACKER_POINTS_CROSS) {
      if(xi < 0 || xi >= img.cols || yi < 0 || yi >= img.rows) {
        measured.push_back(-1e6);
      } else {
        measured.push_back(interpolate(img, xi, yi));
      }
    }
  }
}

void extractMeanAndSquare(vector<float> &vs, float &v_mean, float &v_square) {
  v_mean = mean(vs.begin(), vs.end());
  for(auto &v: vs) v -= v_mean;
  v_square = meanDotSquare(vs.begin(), vs.end(), 0.0);
}

HandDescription measureObject(cv::Mat &img, int sx, int sy, int ex, int ey) {
  HandDescription result;

  if(ex < sx) {
    int tmp = ex;
    ex = sx;
    sx = tmp;
  }
  if(ey < sy) {
    int tmp = ey;
    ey = sy;
    sy = tmp;
  }

  float w = ex - sx; result.w = w;
  float h = ey - sy; result.h = h;

  float cx = (sx + ex) / 2;
  float cy = (sy + ey) / 2;

  sampleFinger(result.fingerSignature, result, img, cx, cy, 1);
  extractMeanAndSquare(result.fingerSignature,
      result.fingerSignatureMean, result.fingerSignatureSquare);
  sampleCross(result.crossSignature, result, img, cx, cy, 1);
  extractMeanAndSquare(result.crossSignature,
      result.crossSignatureMean, result.crossSignatureSquare);

  return result;
}

struct FingerMatchQuality {
  bool windowed;
  float score;
  float scale;
  float beta, gamma;
  float x, y;
  float w, h;

  FingerMatchQuality() {
    score = 0;
    scale = 0;
    windowed = false;
  }
};

void trackObject(cv::Mat &img, const HandDescription &templ, FingerMatchQuality &match,
    HandDescription &track, FingerMatchQuality &trackMatch,
    const ThresholdConfiguration &thresholds) {

  float bestCorr = 0;

  for(float dy = -2; dy < 3; ++dy) {
    for(float dx = -2; dx < 3; ++dx) {
      HandDescription candidate = measureObject(img,
          dx + match.x - match.w / 2 + 0.1, dy + match.y - match.h / 2 + 0.1,
          dx + match.x + match.w / 2 - 0.1, dy + match.y + match.h / 2 - 0.1);

      auto fingerCorr = correlationCoefficientLeftConstRightConst(
          templ.fingerSignature.begin(), templ.fingerSignature.end(),
          templ.fingerSignatureMean, templ.fingerSignatureSquare,
          candidate.fingerSignature.begin(), candidate.fingerSignature.end(),
          candidate.fingerSignatureMean, candidate.fingerSignatureSquare,
          thresholds.contrastThreshold, thresholds.brightnessThreshold);
      auto crossCorr = correlationCoefficientLeftConstRightConst(
          templ.crossSignature.begin(), templ.crossSignature.end(),
          templ.crossSignatureMean, templ.crossSignatureSquare,
          candidate.crossSignature.begin(), candidate.crossSignature.end(),
          candidate.crossSignatureMean, candidate.crossSignatureSquare,
          thresholds.crossContrastThreshold, thresholds.brightnessThreshold);
      float corr = fingerCorr.corr + crossCorr.corr;

      if(corr > bestCorr) {
        bestCorr = corr;

        track = candidate;
        trackMatch = match;
        trackMatch.x += dx;
        trackMatch.y += dy;
      }
    }
  }


  __android_log_print(ANDROID_LOG_INFO, "Gesture", "Tracking quality %f", bestCorr);

  if(bestCorr < thresholds.trackingQualityThreshold) {
    __android_log_print(ANDROID_LOG_INFO, "Gesture", "Tracking lost.");
    track = HandDescription();
    trackMatch = FingerMatchQuality();
  }
}

FingerMatchQuality compareFingers(const HandDescription &obj,
    cv::Mat &img, int x, int y,
    const ThresholdConfiguration &thresholds) {
  FingerMatchQuality result;

  for(float scale = 0.75; scale < 1.5; scale *= 1.1) {
    static vector<float> measured;
    measured.clear();

    sampleFinger(measured, obj, img, x, y, scale);

    auto corrCoeff = correlationCoefficientLeftConst(
      obj.fingerSignature.begin(), obj.fingerSignature.end(),
      obj.fingerSignatureMean, obj.fingerSignatureSquare,
      measured.begin(), measured.end(),
      thresholds.contrastThreshold, thresholds.brightnessThreshold);
    if(fabs(corrCoeff.gamma) > thresholds.brightnessThreshold) {
      break;
    }

    if(fabs(corrCoeff.corr) > result.score) {
      result.score = fabs(corrCoeff.corr);
      result.beta = corrCoeff.beta;
      result.gamma = corrCoeff.gamma;
      result.scale = scale;
      result.x = x;
      result.y = y;
      result.w = obj.w * scale;
      result.h = obj.h * scale;
    }
  }

  return result;
}

struct CrossMatchQuality {
  bool windowed;
  float score;
  float scale;
  float beta, gamma;
  float x, y;
  float w, h;

  CrossMatchQuality() {
    score = 0;
    scale = 0;
    windowed = false;
  }
};

CrossMatchQuality compareCross(const HandDescription &obj,
    cv::Mat &img, int x, int y, float fingerScale,
    const ThresholdConfiguration &thresholds) {
  CrossMatchQuality result;

  for(float scale = fingerScale * 0.8; scale < fingerScale * 1.2; scale *= 1.1) {
    vector<float> measured;

    sampleCross(measured, obj, img, x, y, scale);

    auto corrCoeff = correlationCoefficientLeftConst(
      obj.crossSignature.begin(), obj.crossSignature.end(),
      obj.crossSignatureMean, obj.crossSignatureSquare,
      measured.begin(), measured.end(),
      thresholds.crossContrastThreshold, thresholds.brightnessThreshold);
    if(fabs(corrCoeff.corr) > result.score) {
      result.score = fabs(corrCoeff.corr);
      result.beta = corrCoeff.beta;
      result.gamma = corrCoeff.gamma;
      result.scale = scale;
      result.x = x;
      result.y = y;
      result.w = obj.w * scale;
      result.h = obj.h * scale;
    }
  }

  return result;
}

class HandTracker {
  public:
    HandTracker(int width, int height): cols(width), rows(height) { }

    void setTemplate(cv::Mat &reference, int sx, int sy, int ex, int ey) {
      // FIXME: Must not blur incoming pyramid frame
      GaussianBlur(reference, reference, cv::Size{0,0}, 2, 2);
      templ = measureObject(reference, sx, sy, ex, ey);
      templX = (sx + ex) / 2;
      templY = (sy + ey) / 2;
    }

    FingerMatchQuality searchObject(const HandDescription &obj, cv::Mat &img,
        int sx, int sy, int ex, int ey, const ThresholdConfiguration &thresholds,
        int mgreen, int mblue) {
      FingerMatchQuality best;

      sx = sx < 0? 0: sx;
      sy = sy < 0? 0: sy;
      ex = img.cols < ex? img.cols: ex;
      ey = img.rows < ey? img.rows: ey;

      for(int y = sy; y < ey; y += 3) {
        for(int x = sx; x < ex; x += 3) {
          auto fingers = compareFingers(obj, img, x, y, thresholds);
          if(fingers.score < thresholds.fingersThreshold) continue;

          if(thresholds.searchChecks) {
            auto fingersAbove = compareFingers(obj, img, x, y - (obj.h / 3) * fingers.scale, thresholds);
            if(fingersAbove.score < thresholds.fingersThreshold) continue;
          }

          for(int yy = -1; yy < 2; ++yy) {
            for(int xx = -1; xx < 2; ++xx) {
              auto cross = compareCross(obj, img, x + xx, y + yy, fingers.scale, thresholds);
              cout << fingers.score <<
                "/" << cross.score << "β" << cross.beta << "γ" << cross.gamma <<
                "@" << x << "," << y <<
                "  " << fingers.scale << endl;
              if(cross.score < thresholds.crossThreshold) continue;

              if(fingers.score > best.score) {
                best = fingers;
              }
            }
          }
        }
        // cout << sx << "-" << ex << "," << y << endl;
      }

      if(best.score < thresholds.fingersThreshold) {
        best.score = 0;
      }

      best.windowed =
        best.x < sx + 2 ||
        best.x > ex - 2 ||
        best.y < sy + 2 ||
        best.y > ey - 2;
      return best;
    }

    void handleFrame(cv::Mat &query) {
      // FIXME: Must not blur incoming pyramid frame
      GaussianBlur(query, query, cv::Size{0,0}, 2, 2);
      cv::Mat queryImg(query);

      FingerMatchQuality best;
      for(int window = HAND_TRACKER_WINDOW_MIN; window < HAND_TRACKER_WINDOW_MAX; window *= 2) {
        if((track.w && best.score <= 0) || best.windowed) {
          best = searchObject(track, queryImg,
              trackMatch.x - window,
              trackMatch.y - window,
              trackMatch.x + window,
              trackMatch.y + window,
              TRACKING_THRESHOLDS,
              255, 0);
        }
      }

      if(best.score <= 0 || best.windowed) {
        best = searchObject(templ, queryImg, 0, 0, queryImg.cols, queryImg.rows, SEARCH_THRESHOLDS, 0, 255);
      }

      if(best.score > 0) {
        __android_log_print(ANDROID_LOG_INFO, "Gesture", "Object at: %f,%f", best.x, best.y);

        trackObject(queryImg, templ, best, track, trackMatch, MEASURE_THRESHOLDS);
      } else {
        track = HandDescription();
        trackMatch = FingerMatchQuality();
      }
    }

    bool trackingIsSuccessful() {
      return trackMatch.score != 0;
    }

    void getTransformation(float *transformationData) {
      float scale = trackMatch.w / templ.w;

      // transformationData[y * 3 + x] = ...
      transformationData[0 * 3 + 0] = scale;
      transformationData[0 * 3 + 1] = 0;
      transformationData[0 * 3 + 2] = trackMatch.x - templX * scale;
      transformationData[1 * 3 + 0] = 0;
      transformationData[1 * 3 + 1] = scale;
      transformationData[1 * 3 + 2] = trackMatch.y - templY * scale;
      transformationData[2 * 3 + 0] = 0;
      transformationData[2 * 3 + 1] = 0;
      transformationData[2 * 3 + 2] = 1;
    }

  private:
    HandDescription templ;
    int templX, templY;

    HandDescription track;
    FingerMatchQuality trackMatch;

    int cols, rows;
};

static HandTracker *tracker = nullptr;

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1setMarker
    (JNIEnv *env, jclass, jint sx, jint sy, jint ex, jint ey) {

  if(tracker) delete tracker;
  tracker = new HandTracker(cameraWidth, cameraHeight);

  auto fingerFrame = frameHandler->lastFrame();
  if(!fingerFrame.get()) return;

  tracker->setTemplate(fingerFrame->img_pyr_[0], sx, sy, ex, ey);

  __android_log_print(ANDROID_LOG_INFO, "Gesture", "Tracker template initialized");
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1processFrame
  (JNIEnv *env, jclass, jfloatArray debugImage) {

  auto fingerFrame = frameHandler->lastFrame();
  if(!fingerFrame.get()) return;

  // TODO: If null debug data array is passed, don't copy.
  //       During normal operations, jut pass null then.
  jfloat *debugData = env->GetFloatArrayElements(debugImage, 0);
  for(int y = 0; y < cameraHeight; ++y) {
    for(int x = 0; x < cameraWidth; ++x) {
      debugData[y * cameraWidth + x] = fingerFrame->img_pyr_[0].at<unsigned char>(y, x) / 512.0f;
    }
  }
  env->ReleaseFloatArrayElements(debugImage, debugData, 0);

  if(!tracker) return;

  tracker->handleFrame(fingerFrame->img_pyr_[0]);
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1getTransformationRelative
  (JNIEnv *env, jclass, jlong time_nano, jfloatArray transformation) {

  if(env->IsSameObject(transformation, nullptr)) {
    env->ThrowNew(env->FindClass("java/lang/NullPointerException"),
        "null passed for output buffer");
    return;
  }

  jfloat *transformationData = env->GetFloatArrayElements(transformation, 0);

  if(tracker && tracker->trackingIsSuccessful()) {
    tracker->getTransformation(transformationData);
  } else {
    for(int y = 0; y < 3; ++y) {
      for(int x = 0; x < 3; ++x) {
        transformationData[y * 3 + x] = 0;
      }
    }
  }

  env->ReleaseFloatArrayElements(transformation, transformationData, 0);
}
