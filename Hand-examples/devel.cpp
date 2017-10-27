#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <array>
#include <sstream>
#include <iomanip>

using namespace cv;
using namespace std;

struct __attribute__((__packed__)) rgb {
  unsigned char b, g, r;
};

struct ObjectDescription {
  float w, h;
  std::vector<float> fingerSignature;
  float fingerSignatureMean, fingerSignatureSquare;

  std::vector<float> crossSignature;
  float crossSignatureMean, crossSignatureSquare;

  ObjectDescription() {
    w = 0;
  }
};

const bool CIRATEFI_POSITIVE_ONLY = true;
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

float interpolate(Mat &img, float x, float y, unsigned char) {
  return img.at<rgb>(y + 0.5, x + 0.5).r;
}

// float interpolate(Mat &img, float x, float y, unsigned char draw) {
//   img.at<rgb>(y, x).b = std::max(img.at<rgb>(y, x).b, draw);
// 
//   int lx = x;
//   int ly = y;
//   if(lx < 0 || ly < 0) {
//     return -1e6;
//   }
// 
//   int hx = lx + 1;
//   int hy = ly + 1;
//   if(hx >= img.cols || hy >= img.rows) {
//     return -1e6;
//   }
// 
//   float weight_x = x - lx;
//   float weight_y = y - ly;
// 
//   float v00 = img.at<rgb>(ly, lx).r;
//   float v10 = img.at<rgb>(hy, lx).r;
//   float v01 = img.at<rgb>(ly, hx).r;
//   float v11 = img.at<rgb>(hy, hx).r;
// 
//   float lcol = weight_y * v10 + (1 - weight_y) * v00;
//   float hcol = weight_y * v11 + (1 - weight_y) * v01;
// 
//   float result = weight_x * hcol + (1 - weight_x) * lcol;
//   return result;
// }

class Image {
  public:
    Mat data;
    // Mat interpolationCache;
    int rows;
    int cols;

  public:
    explicit Image(Mat &img) {
      data = img;
      rows = img.rows;
      cols = img.cols;
      // interpolationCache = Mat(data.rows * CIRATEFI_INTERPOLATION_SCALE + CIRATEFI_INTERPOLATION_SCALE, data.cols * CIRATEFI_INTERPOLATION_SCALE + CIRATEFI_INTERPOLATION_SCALE, CV_8UC1, 0.0);

      // for(int y = 0; y < img.rows - 1; ++y) {
      //   const int ly = y;
      //   const int hy = y + 1;

      //   for(int x = 0; x < img.cols - 1; ++x) {
      //     const int lx = x;
      //     const int hx = x + 1;

      //     const float v00 = img.at<rgb>(ly, lx).r;
      //     const float v10 = img.at<rgb>(hy, lx).r;
      //     const float v01 = img.at<rgb>(ly, hx).r;
      //     const float v11 = img.at<rgb>(hy, hx).r;

      //     unsigned char *block = &interpolationCache.at<unsigned char>(y * CIRATEFI_INTERPOLATION_SCALE, x * CIRATEFI_INTERPOLATION_SCALE);

      //     const float dyl = (v10 - v00) / CIRATEFI_INTERPOLATION_SCALE;
      //     const float dyh = (v11 - v01) / CIRATEFI_INTERPOLATION_SCALE;

      //     float lcol = v00;
      //     float hcol = v01;
      //     for(int yy = 0; yy < CIRATEFI_INTERPOLATION_SCALE; ++yy) {
      //       const float dx = (hcol - lcol) / CIRATEFI_INTERPOLATION_SCALE;

      //       float v = lcol;
      //       for(unsigned char *const end = block + CIRATEFI_INTERPOLATION_SCALE;
      //           block < end; ++block, v += dx) {
      //         *block = v;
      //       }

      //       lcol += dyl;
      //       hcol += dyh;
      //       block += interpolationCache.cols - CIRATEFI_INTERPOLATION_SCALE;
      //     }
      //   }
      // }
    }
};

// float interpolate(Image &img, float x, float y, unsigned char) {
//   unsigned char result = img.interpolationCache.at<unsigned char>(y * CIRATEFI_INTERPOLATION_SCALE, x * CIRATEFI_INTERPOLATION_SCALE); 
//   if(result) return result;
// 
//   result = interpolate(img.data, (int)(x * CIRATEFI_INTERPOLATION_SCALE) / static_cast<float>(CIRATEFI_INTERPOLATION_SCALE), (int)(y * CIRATEFI_INTERPOLATION_SCALE) / static_cast<float>(CIRATEFI_INTERPOLATION_SCALE), 0);
//   img.interpolationCache.at<unsigned char>(y * CIRATEFI_INTERPOLATION_SCALE, x * CIRATEFI_INTERPOLATION_SCALE) = result? result: 1;
//   return result;
// }

// float interpolate(Image &img, float x, float y, unsigned char) {
//   if(x < 0 || y < 0 || x >= img.cols || y >= img.rows) {
//     return -1e6;
//   }
// 
//   return img.interpolationCache.at<unsigned char>(y * CIRATEFI_INTERPOLATION_SCALE, x * CIRATEFI_INTERPOLATION_SCALE); 
// }

template<typename I> typename std::iterator_traits<I>::value_type mean(
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
typename std::iterator_traits<I>::value_type meanDotProduct(
    const I &is, const I &ie, const IT &im, const J &js, const J &, const JT &jm) {
  typename std::iterator_traits<I>::value_type p = 0;

  auto i = is;
  auto j = js;

  while(i != ie) {
    p += (*i - im) * (*j - jm);
    ++i; ++j;
  }

  return p;
}

template<typename I, typename IT>
typename std::iterator_traits<I>::value_type meanDotSquare(
    const I &is, const I &ie, const IT &im) {
  typename std::iterator_traits<I>::value_type p = 0;

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

template<typename I, typename J>
CorrelationCoefficient<typename I::value_type> correlationCoefficient(
    const I &is, const I &ie, const J &js, const J &je,
    const float &contrastThreshold, const float &brightnessThreshold) {
  struct CorrelationCoefficient<typename I::value_type> result;

  const typename I::value_type xm = mean(is, ie);
  const typename std::iterator_traits<J>::value_type ym = mean(js, je);

  const typename I::value_type xx = meanDotSquare(is, ie, xm);
  const typename I::value_type xy = meanDotProduct(is, ie, xm, js, je, ym);

  float beta = xy / xx;
  float gamma = ym - beta * xm;

  result.beta = beta;
  result.gamma = gamma;

  if(CIRATEFI_POSITIVE_ONLY && beta < 0) {
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

  const typename std::iterator_traits<J>::value_type yy = meanDotSquare(js, je, ym);
  result.corr = xy / sqrt(xx * yy);

  return result;
}

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

  if(CIRATEFI_POSITIVE_ONLY && beta < 0) {
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
CorrelationCoefficient<typename I::value_type> correlationCoefficientLeftConstRightMean(
    const I &is, const I &ie, const typename I::value_type im, const typename I::value_type i2,
    const J &js, const J &je, const typename J::value_type jm,
    const ThresholdConfiguration &thresholds) {
  struct CorrelationCoefficient<typename I::value_type> result;

  const typename iterator_traits<I>::value_type xm = im;
  const typename iterator_traits<J>::value_type ym = jm;

  const typename I::value_type xx = i2;
  const typename I::value_type xy = meanDotProductLeftConst(is, ie, js, je, ym);

  float beta = xy / xx;
  float gamma = ym - beta * xm;

  result.beta = beta;
  result.gamma = gamma;

  if(CIRATEFI_POSITIVE_ONLY && beta < 0) {
    result.corr = 0;
    return result;
  }

  if(fabs(beta) < thresholds.contrastThreshold || 1 / thresholds.contrastThreshold < fabs(beta)) {
    result.corr = 0;
    return result;
  }

  if(fabs(gamma) > thresholds.brightnessThreshold) {
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

  if(CIRATEFI_POSITIVE_ONLY && beta < 0) {
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

void sampleFinger(std::vector<float> &measured, const ObjectDescription &obj,
    Image &img, int x, int y, float scale) {
  float xi = x - (obj.w / 2 * scale);
  const float dx = obj.w * scale / HAND_TRACKER_POINTS;
  for(int i = 0; i < HAND_TRACKER_POINTS; ++i, xi += dx) {
    if(xi < 0 || xi >= img.cols) {
      measured.push_back(-1e6);
    } else {
      measured.push_back(interpolate(img.data, xi, y, 0));
    }
  }
}

void sampleCross(std::vector<float> &measured, const ObjectDescription &obj,
    Image &img, int x, int y, float scale) {
  float xi = x - (obj.w / 2 * scale);
  for(int i = 0; i < HAND_TRACKER_POINTS; ++i, xi += obj.w * scale / HAND_TRACKER_POINTS_CROSS) {
    float yi = y - (obj.h / 2 * scale);
    for(int j = 0; j < HAND_TRACKER_POINTS; ++j, yi += obj.h * scale / HAND_TRACKER_POINTS_CROSS) {
      if(xi < 0 || xi >= img.cols || yi < 0 || yi >= img.rows) {
        measured.push_back(-1e6);
      } else {
        measured.push_back(interpolate(img.data, xi, yi, 0));
      }
    }
  }
}

void extractMeanAndSquare(std::vector<float> &vs, float &v_mean, float &v_square) {
  v_mean = mean(vs.begin(), vs.end());
  for(auto &v: vs) v -= v_mean;
  v_square = meanDotSquare(vs.begin(), vs.end(), 0.0);
}

ObjectDescription measureObject(Image &img, int sx, int sy, int ex, int ey) {
  ObjectDescription result;

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

// unsigned int querySum(unsigned char *center, const ObjectDescription::DeltaCoords *deltas, int n) {
//   unsigned int sum_0 = 0;
//   unsigned int sum_1 = 0;
//   for(const ObjectDescription::DeltaCoords *e = deltas + (n >> 1 << 1); deltas != e; deltas += 2) {
//     sum_0 += *(center + deltas[0].d);
//     sum_1 += *(center + deltas[1].d);
//   }
//   unsigned int sum = sum_0 + sum_1;
// 
//   if(n & 1) {
//     sum += *(center + deltas[0].d);
//   }
// 
//   return sum;
// }

struct FingerMatchQuality {
  bool windowed;
  float score;
  float scale;
  int startQuery;
  float beta, gamma;
  float x, y;
  float w, h;

  FingerMatchQuality() {
    score = 0;
    scale = 0;
    windowed = false;
  }
};

void trackObject(Image &img, const ObjectDescription &templ, FingerMatchQuality &match,
    ObjectDescription &track, FingerMatchQuality &trackMatch,
    const ThresholdConfiguration &thresholds) {

  float bestCorr = 0;

  cout << "TRACKING" << endl;
  for(float dy = -2; dy < 3; ++dy) {
    for(float dx = -2; dx < 3; ++dx) {
      ObjectDescription candidate = measureObject(img,
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

      cout << fingerCorr.corr << ","
        << crossCorr.corr << "." << crossCorr.beta << "." << crossCorr.gamma << endl;

      if(corr > bestCorr) {
        bestCorr = corr;

        track = candidate;
        trackMatch = match;
        trackMatch.x += dx;
        trackMatch.y += dy;
      }
    }
  }

  cout << "tracking quality: " << bestCorr << endl;

  if(bestCorr < thresholds.trackingQualityThreshold) {
    cout << "tracking lost" << endl;
    track = ObjectDescription();
  }
}

FingerMatchQuality compareFingers(const ObjectDescription &obj,
    Image &img, int x, int y,
    const ThresholdConfiguration &thresholds) {
  FingerMatchQuality result;

  for(float scale = 0.75; scale < 1.5; scale *= 1.1) {
    static std::vector<float> measured;
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
  int startQuery;
  float beta, gamma;
  float x, y;
  float w, h;

  CrossMatchQuality() {
    score = 0;
    scale = 0;
    windowed = false;
  }
};

CrossMatchQuality compareCross(const ObjectDescription &obj,
    Image &img, int x, int y, float fingerScale,
    const ThresholdConfiguration &thresholds) {
  CrossMatchQuality result;

  for(float scale = fingerScale * 0.8; scale < fingerScale * 1.2; scale *= 1.1) {
    std::vector<float> measured;

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

// void drawMatch(const ObjectDescription &obj,
//     Mat &img, const AllMatchQuality &match, int, int g, int b) {
//   int sx = match.x - (obj.w / 2.0) * match.wx - (obj.h / 2.0) * match.hx;
//   int sy = match.y - (obj.w / 2.0) * match.wy - (obj.h / 2.0) * match.hy;
// 
//   for(int ly = 0; ly < obj.h; ++ly) {
//     for(int lx = 0; lx < obj.w; ++lx) {
//       float ix = sx + lx * match.wx + ly * match.hx;
//       float iy = sy + lx * match.wy + ly * match.hy;
// 
//       if(iy > 0 && ix > 0 && iy < img.rows && ix < img.cols) {
//         img.at<rgb>(iy, ix).g = g;
//         img.at<rgb>(iy, ix).b = b;
//       }
//     }
//   }
// }

// std::vector<Image> buildImagePyramid(Image img) {
//   std::vector<Image> ret;
//   ret.push_back(img);
// 
//   for(int n = 0; n < CIRATEFI_PYRAMID_LEVELS; ++n) {
//     Image &cur = ret.back();
//     Mat next(cur.rows / 2, cur.cols / 2, CV_8UC3);
// 
//     for(int y = 0; y < next.rows; ++y) {
//       for(int x = 0; x < next.cols; ++x) {
//         unsigned char val = (
//           static_cast<unsigned int>(cur.data.at<rgb>(y * 2, x * 2).r) +
//           static_cast<unsigned int>(cur.data.at<rgb>(y * 2, x * 2 + 1).r) +
//           static_cast<unsigned int>(cur.data.at<rgb>(y * 2 + 1, x * 2).r) +
//           static_cast<unsigned int>(cur.data.at<rgb>(y * 2 + 1, x * 2 + 1).r)
//         ) / 4;
//         next.at<rgb>(y, x) = rgb{ val, val, val };
//       }
//     }
// 
//     ret.push_back(Image(next));
//   }
// 
//   return ret;
// }
// 
// template<class F> Mat buildExtremumImage(Mat &img, int range, const F &f) {
//   Mat ret = img.clone();
// 
//   for(int i = 0; i < range; ++i) {
//     Mat next = ret.clone();
//     for(int y = 0; y < ret.rows; ++y) {
//       for(int x = 0; x < ret.cols; ++x) {
//         unsigned char ext = ret.at<rgb>(y, x).r;
// 
//         // if(x > 0 && y > 0)                   ext = f(ext, ret.at<rgb>(y - 1, x - 1).r);
//         if(x > 0)                            ext = f(ext, ret.at<rgb>(y,     x - 1).r);
//         // if(x > 0 && y < img.rows - 1)        ext = f(ext, ret.at<rgb>(y + 1, x - 1).r);
//         if(y > 0)                            ext = f(ext, ret.at<rgb>(y - 1, x).r);
//         if(y < img.rows - 1)                 ext = f(ext, ret.at<rgb>(y + 1, x).r);
//         //if(x < img.cols && y > 0)            ext = f(ext, ret.at<rgb>(y - 1, x + 1).r);
//         if(x < img.cols)                     ext = f(ext, ret.at<rgb>(y,     x + 1).r);
//         // if(x < img.cols && y < img.rows - 1) ext = f(ext, ret.at<rgb>(y + 1, x + 1).r);
// 
//         next.at<rgb>(y, x).r = ext;
//       }
//     }
// 
//     ret = next;
//   }
// 
//   return ret;
// }

class CiratefiTracker {
  public:
    CiratefiTracker(int width, int height): cols(width), rows(height) { }

    void setTemplate(Mat &reference, int sx, int sy, int ex, int ey) {
      GaussianBlur(reference, reference, cv::Size{0,0}, 2, 2);

      Image referenceImage(reference);
      templ = measureObject(referenceImage, sx, sy, ex, ey);
    }

    FingerMatchQuality searchObject(const ObjectDescription &obj, Image &img,
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

          img.data.at<rgb>(y, x).g = mgreen / 4;
          img.data.at<rgb>(y, x).b = mblue / 4;

          if(thresholds.searchChecks) {
            auto fingersAbove = compareFingers(obj, img, x, y - (obj.h / 3) * fingers.scale, thresholds);
            if(fingersAbove.score < thresholds.fingersThreshold) continue;
          }

          img.data.at<rgb>(y, x).g = mgreen / 2;
          img.data.at<rgb>(y, x).b = mblue / 2;

          for(int yy = -1; yy < 2; ++yy) {
            for(int xx = -1; xx < 2; ++xx) {
              auto cross = compareCross(obj, img, x + xx, y + yy, fingers.scale, thresholds);
              cout << fingers.score <<
                "/" << cross.score << "β" << cross.beta << "γ" << cross.gamma <<
                "@" << x << "," << y <<
                "  " << fingers.scale << endl;
              if(cross.score < thresholds.crossThreshold) continue;

              img.data.at<rgb>(y + yy, x + xx).g = mgreen;
              img.data.at<rgb>(y + yy, x + xx).b = mblue;

              if(fingers.score > best.score) {
                best = fingers;
              }
            }
          }
        }
        // cout << sx << "-" << ex << "," << y << endl;
      }

      cout << "BEST SCORE: " << best.score << endl;
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

    void handleFrame(Mat &query) {
      GaussianBlur(query, query, cv::Size{0,0}, 2, 2);
      Image queryImg(query);

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

      // if(track.w && best.score <= 0) {
      //   best = searchObject(track, queryImg, 0, 0, queryImg.cols, queryImg.rows, SEARCH_THRESHOLDS, 255, 255);
      // }
      if(best.score <= 0 || best.windowed) {
        best = searchObject(templ, queryImg, 0, 0, queryImg.cols, queryImg.rows, SEARCH_THRESHOLDS, 0, 255);
      }

      if(best.score > 0) {
        trackObject(queryImg, templ, best, track, trackMatch, MEASURE_THRESHOLDS);

        rectangle(queryImg.data,
                  Point(best.x - best.w / 2, best.y - best.h / 2),
                  Point(best.x + best.w / 2, best.y + best.h / 2),
                  Scalar(0, 0, 255),
                  1,
                  8);
      } else {
        track = ObjectDescription();
      }

      // imshow("Debug", queryImg.data);

      // for(int i = 0; i < 5; ++i) {
      //   if( waitKey(10) >= 0 ) {
      //     throw 0;
      //   }
      // }
    }

  private:
    ObjectDescription templ;

    ObjectDescription track;
    FingerMatchQuality trackMatch;

    int cols, rows;
};

int main(int, char**)
{
    // namedWindow("Object Reference", WINDOW_AUTOSIZE);
    // namedWindow("Debug", WINDOW_AUTOSIZE);

    // Mat reference = imread("svo.0120.png", IMREAD_COLOR );
    // if(reference.empty()) {
    //     cout <<  "Could not load reference image" << endl ;
    //     return -1;
    // }

    // int sx = 360;
    // int ex = 420;
    // int sy = 130;
    // int ey = 175;

    // rectangle(reference,
    //           Point(sx - 2, sy - 2),
    //           Point(ex + 2, ey + 2),
    //           Scalar(0, 255, 0),
    //           1,
    //           8);

    // imshow("Object Reference", reference);

    // CiratefiTracker tracker(reference.cols, reference.rows);
    // tracker.setTemplate(reference, sx, sy, ex, ey);

    Mat reference = imread("svo.0020.png", IMREAD_COLOR );
    if(reference.empty()) {
        cout <<  "Could not load reference image" << endl ;
        return -1;
    }

    // int sx = 278;
    // int ex = 393;
    // int sy = 130;
    // int ey = 225;

    int sx = 278;
    int ex = 393;
    int sy = 130;
    int ey = 185;

    rectangle(reference,
              Point(sx - 2, sy - 2),
              Point(ex + 2, ey + 2),
              Scalar(0, 255, 0),
              1,
              8);

    // Mat test(reference.rows * 12, reference.cols * 12, CV_8UC1, 0.0);
    // for(float y = 0; y < test.rows; ++y) {
    //   for(float x = 0; x < test.cols; ++x) {
    //     // test.at<unsigned char>(y, x) = interpolate(reference, x / 12, y / 12, 0);
    //     // test.at<unsigned char>(y, x) = x;
    //   }
    // }

    // imshow("Object Reference", test);
    // waitKey(100000);

    CiratefiTracker tracker(reference.cols, reference.rows);
    tracker.setTemplate(reference, sx, sy, ex, ey);

    // imshow("Object Reference", reference);

    // for(int i = 20; i < 25; ++i) {
    for(int i = 1; i < 199; ++i) {
      ostringstream filename;
      filename << "svo." << setw(4) << setfill('0') << i << ".png";

      Mat query = imread(filename.str(), IMREAD_COLOR );
      // Mat query = imread("svo.0120.png", IMREAD_COLOR );
      if(query.empty()) {
          cout <<  "Could not load " << filename.str() << endl ;
          return -1;
      }

      tracker.handleFrame(query);

      imwrite("out/" + filename.str(), query);
    }

    return 0;
}
