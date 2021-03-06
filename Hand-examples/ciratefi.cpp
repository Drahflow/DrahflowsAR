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
  // Actual object description
  float w, h;
  std::vector<float> circleValuesCoarse;
  float circleValuesCoarseMean, circleValuesCoarseSquared;
  std::vector<float> circleValues;
  float circleValuesMean, circleValuesSquared;
  std::vector<float> radialValues;
  float radialValuesMean, radialValuesSquared;
  std::vector<float> orthogonalValues;
  std::vector<float> parallelValues;
  std::vector<float> allValues;
  float allValuesRange;

  // Cached calculations which depend on object geometry
  struct DeltaCoords {
    ptrdiff_t d;
  };

  struct CircleQuery {
    float r;
    int pyrLvl;
    std::vector<DeltaCoords> samplePoints;
  };

  std::vector<CircleQuery> coarseCircleQueries;
  float coarseCircleRMax;

  std::vector<CircleQuery> pyramidCircleQueries;
  float pyramidCircleRMax;

  std::vector<CircleQuery> circleQueries;
  float circleRMax;

  ObjectDescription() {
    w = h = 0;
  }
};

const bool CIRATEFI_POSITIVE_ONLY = true;
const int CIRATEFI_INTERPOLATION_SCALE = 12;
const int CIRATEFI_PYRAMID_LEVELS = 6;
const int CIRATEFI_CIRCLES_COARSE = 5;
const int CIRATEFI_CIRCLES = 15;
const int CIRATEFI_RADIALS = 32;
const int CIRATEFI_ORTHOGONALS = 16;
const int CIRATEFI_PARALLELS = 16;
const float CIRATEFI_CIRCLE_DIVISOR_COARSE = 1.22;
const float CIRATEFI_CIRCLE_DIVISOR = 1.07;
const float CIRATEFI_CIRCLE_SHIFT = 1.007;
const float CIRATEFI_CIRCLE_DENSITY = 0.5;
const float CIRATEFI_RADIAL_DENSITY = 0.3;
const float CIRATEFI_MAGNIFICATION_FACTOR = 3;
const float CIRATEFI_MINIFICATION_FACTOR = 0.3;
//const float CIRATEFI_CONTRAST_THRESHOLD = 0.3;
//const float CIRATEFI_BRIGHTNESS_THRESHOLD = 75;
//const float CIRATEFI_CIRCLE_THRESHOLD_COARSE = 0.60;
//const float CIRATEFI_CIRCLE_THRESHOLD_PYRAMID = 0.65;
//const float CIRATEFI_CIRCLE_THRESHOLD = 0.70;
//const float CIRATEFI_CIRCLE_THRESHOLD_SHIFTED = 0.75;
//const float CIRATEFI_RADIAL_THRESHOLD = 0.80;
//const float CIRATEFI_ORTHOGONAL_THRESHOLD = 0.55;
//const float CIRATEFI_PARALLEL_THRESHOLD = 0.55;
//const float CIRATEFI_ALL_EARLY_THRESHOLD = 0.50;
//const float CIRATEFI_ALL_THRESHOLD = 0.55;
const float CIRATEFI_ALL_STEPS = 0.26;
const float CIRATEFI_FINAL_WIGGLES = 30;
const float CIRATEFI_FINAL_WIGGLE_STEP = 0.1;

struct ThresholdConfiguration {
  public:
    float contrastThreshold;
    float brightnessThreshold;
    float circleThresholdCoarse;
    float circleThresholdPyramid;
    float circleThreshold;
    float circleThresholdShifted;
    float radialThreshold;
    float orthogonalThreshold;
    float parallelThreshold;
    float allEarlyThreshold;
    float allThreshold;
};

const ThresholdConfiguration VERIFY_THRESHOLDS = {
  contrastThreshold: 0.3,
  brightnessThreshold: 75,
  circleThresholdCoarse: 0.30,
  circleThresholdPyramid: 0.65,
  circleThreshold: 0.40,
  circleThresholdShifted: 0.55,
  radialThreshold: 0.60,
  orthogonalThreshold: 0.35,
  parallelThreshold: 0.35,
  allEarlyThreshold: 0.30,
  allThreshold: 0.35
};

const ThresholdConfiguration SEARCH_THRESHOLDS = {
  contrastThreshold: 0.3,
  brightnessThreshold: 75,
  circleThresholdCoarse: 0.60,
  circleThresholdPyramid: 0.65,
  circleThreshold: 0.70,
  circleThresholdShifted: 0.75,
  radialThreshold: 0.80,
  orthogonalThreshold: 0.55,
  parallelThreshold: 0.55,
  allEarlyThreshold: 0.50,
  allThreshold: 0.55
};

const ThresholdConfiguration TRACKING_THRESHOLDS = {
  contrastThreshold: 0.3,
  brightnessThreshold: 75,
  circleThresholdCoarse: 0.60,
  circleThresholdPyramid: 0.65,
  circleThreshold: 0.70,
  circleThresholdShifted: 0.75,
  radialThreshold: 0.80,
  orthogonalThreshold: 0.55,
  parallelThreshold: 0.55,
  allEarlyThreshold: 0.50,
  allThreshold: 0.55
};

const ThresholdConfiguration FAST_TRACKING_THRESHOLDS = {
  contrastThreshold: 0.3,
  brightnessThreshold: 75,
  circleThresholdCoarse: 0.95,
  circleThresholdPyramid: 0.95,
  circleThreshold: 0.95,
  circleThresholdShifted: 0.95,
  radialThreshold: 0.90,
  orthogonalThreshold: 0.75,
  parallelThreshold: 0.75,
  allEarlyThreshold: 0.50,
  allThreshold: 0.75
};

float interpolate(Mat &img, float x, float y, unsigned char) {
  int lx = x;
  int ly = y;
  if(lx < 0 || ly < 0) {
    return -1e6;
  }

  int hx = lx + 1;
  int hy = ly + 1;
  if(hx >= img.cols || hy >= img.rows) {
    return -1e6;
  }

  float weight_x = x - lx;
  float weight_y = y - ly;

  float v00 = img.at<rgb>(ly, lx).r;
  float v10 = img.at<rgb>(hy, lx).r;
  float v01 = img.at<rgb>(ly, hx).r;
  float v11 = img.at<rgb>(hy, hx).r;

  float lcol = weight_y * v10 + (1 - weight_y) * v00;
  float hcol = weight_y * v11 + (1 - weight_y) * v01;

  return weight_x * hcol + (1 - weight_x) * lcol;
}

class Image {
  public:
    Mat data;
    Mat interpolationCache;
    int rows;
    int cols;

  public:
    explicit Image(Mat &img) {
      data = img.clone();
      rows = img.rows;
      cols = img.cols;
      interpolationCache = Mat(data.rows * CIRATEFI_INTERPOLATION_SCALE + CIRATEFI_INTERPOLATION_SCALE, data.cols * CIRATEFI_INTERPOLATION_SCALE + CIRATEFI_INTERPOLATION_SCALE, CV_8UC1, 0.0);

      for(int y = 0; y < img.rows - 1; ++y) {
        const int ly = y;
        const int hy = y + 1;

        for(int x = 0; x < img.cols - 1; ++x) {
          const int lx = x;
          const int hx = x + 1;

          const float v00 = img.at<rgb>(ly, lx).r;
          const float v10 = img.at<rgb>(hy, lx).r;
          const float v01 = img.at<rgb>(ly, hx).r;
          const float v11 = img.at<rgb>(hy, hx).r;

          unsigned char *block = &interpolationCache.at<unsigned char>(y * CIRATEFI_INTERPOLATION_SCALE, x * CIRATEFI_INTERPOLATION_SCALE);

          const float dyl = (v10 - v00) / CIRATEFI_INTERPOLATION_SCALE;
          const float dyh = (v11 - v01) / CIRATEFI_INTERPOLATION_SCALE;

          float lcol = v00;
          float hcol = v01;
          for(int yy = 0; yy < CIRATEFI_INTERPOLATION_SCALE; ++yy) {
            const float dx = (hcol - lcol) / CIRATEFI_INTERPOLATION_SCALE;

            float v = lcol;
            for(unsigned char *const end = block + CIRATEFI_INTERPOLATION_SCALE;
                block < end; ++block, v += dx) {
              *block = v;
            }

            lcol += dyl;
            hcol += dyh;
            block += interpolationCache.cols - CIRATEFI_INTERPOLATION_SCALE;
          }
        }
      }

      // for(float y = 0; y < img.rows * CIRATEFI_INTERPOLATION_SCALE; ++y) {
      //   for(float x = 0; x < img.cols * CIRATEFI_INTERPOLATION_SCALE; ++x) {
      //     unsigned char expected = interpolate(img, x / CIRATEFI_INTERPOLATION_SCALE, y / CIRATEFI_INTERPOLATION_SCALE, 0);
      //     if(fabs(interpolationCache.at<unsigned char>(y, x) - expected) > 1) {
      //       cout << x << "," << y << endl;
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

float interpolate(Image &img, float x, float y, unsigned char) {
  if(x < 0 || y < 0 || x >= img.cols || y >= img.rows) {
    return -1e6;
  }

  return img.interpolationCache.at<unsigned char>(y * CIRATEFI_INTERPOLATION_SCALE, x * CIRATEFI_INTERPOLATION_SCALE); 
}

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
    const ThresholdConfiguration &thresholds) {
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

  if(fabs(beta) < thresholds.contrastThreshold || 1 / thresholds.contrastThreshold < fabs(beta)) {
    result.corr = 0;
    return result;
  }

  if(fabs(gamma) > thresholds.brightnessThreshold) {
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
    const ThresholdConfiguration &thresholds) {
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
CorrelationCoefficient<typename I::value_type> correlationCoefficientLeftConstRightMean(
    const I &is, const I &ie, const typename I::value_type im, const typename I::value_type i2,
    const J &js, const J &je, const typename J::value_type jm,
    const ThresholdConfiguration &thresholds) {
  struct CorrelationCoefficient<typename I::value_type> result;

  const typename iterator_traits<I>::value_type xm = im;
  const typename iterator_traits<J>::value_type ym = jm;

  if(im == 12345451) cout << "!" << endl;
  if(jm == 12345451) cout << "!" << endl;

  const typename I::value_type xx = i2;
  const typename I::value_type xy = meanDotProductLeftConst(is, ie, js, je, ym);

  if(xx == 12345451) cout << "!" << endl;
  if(xy == 12345451) cout << "!" << endl;

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
    const ThresholdConfiguration &thresholds) {
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

  if(fabs(beta) < thresholds.contrastThreshold || 1 / thresholds.contrastThreshold < fabs(beta)) {
    result.corr = 0;
    return result;
  }

  if(fabs(gamma) > thresholds.brightnessThreshold) {
    result.corr = 0;
    return result;
  }

  const typename iterator_traits<J>::value_type yy = j2;
  result.corr = xy / sqrt(xx * yy);

  return result;
}

void precomputeCoarseCircleQueries(Image &img, ObjectDescription &obj, float maxR) {
  float w = obj.w;
  float h = obj.h;

  float r = (w < h? w: h) / 2;
  float rs = r * CIRATEFI_MAGNIFICATION_FACTOR;
  float re = r * CIRATEFI_MINIFICATION_FACTOR / pow(CIRATEFI_CIRCLE_DIVISOR_COARSE, CIRATEFI_CIRCLES_COARSE);

  float ri = r;
  for(; ri < rs; ri *= CIRATEFI_CIRCLE_DIVISOR_COARSE);
  obj.coarseCircleRMax = ri;
  for(; ri > re; ri /= CIRATEFI_CIRCLE_DIVISOR_COARSE) {
    ObjectDescription::CircleQuery query;
    query.r = ri;

    float rj = ri;
    int pyrLvl = 0;
    while(rj > maxR && pyrLvl < CIRATEFI_PYRAMID_LEVELS - 1) {
      rj /= 2;
      pyrLvl++;
    }

    query.pyrLvl = pyrLvl;

    float len = 2 * M_PI * rj;
    for(float alpha = 0; alpha < 2 * M_PI; alpha += 2 * M_PI / (len * CIRATEFI_CIRCLE_DENSITY)) {
      query.samplePoints.push_back(ObjectDescription::DeltaCoords{
        static_cast<int>(CIRATEFI_INTERPOLATION_SCALE * rj * cosf(alpha)) *
        CIRATEFI_INTERPOLATION_SCALE * (1 + (img.data.cols >> pyrLvl)) +
        static_cast<int>(CIRATEFI_INTERPOLATION_SCALE * rj * sinf(alpha))
      });
    }

    obj.coarseCircleQueries.push_back(query);
  }
}

void precomputePyramidCircleQueries(Image &img, ObjectDescription &obj, float maxR) {
  float w = obj.w;
  float h = obj.h;

  float r = (w < h? w: h) / 2;
  float rs = r * CIRATEFI_MAGNIFICATION_FACTOR;
  float re = r * CIRATEFI_MINIFICATION_FACTOR / pow(CIRATEFI_CIRCLE_DIVISOR, CIRATEFI_CIRCLES);

  float ri = r;
  for(; ri < rs; ri *= CIRATEFI_CIRCLE_DIVISOR);
  obj.pyramidCircleRMax = ri;
  for(; ri > re; ri /= CIRATEFI_CIRCLE_DIVISOR) {
    ObjectDescription::CircleQuery query;
    query.r = ri;

    float rj = ri;
    int pyrLvl = 0;
    while(rj > maxR && pyrLvl < CIRATEFI_PYRAMID_LEVELS - 1) {
      rj /= 2;
      pyrLvl++;
    }

    query.pyrLvl = pyrLvl;

    float len = 2 * M_PI * rj;
    for(float alpha = 0; alpha < 2 * M_PI; alpha += 2 * M_PI / (len * CIRATEFI_CIRCLE_DENSITY)) {
      query.samplePoints.push_back(ObjectDescription::DeltaCoords{
        static_cast<int>(CIRATEFI_INTERPOLATION_SCALE * rj * cosf(alpha)) *
        CIRATEFI_INTERPOLATION_SCALE * (1 + (img.data.cols >> pyrLvl)) +
        static_cast<int>(CIRATEFI_INTERPOLATION_SCALE * rj * sinf(alpha))
      });
    }

    obj.pyramidCircleQueries.push_back(query);
  }
}

void precomputeCircleQueries(Image &img, ObjectDescription &obj) {
  float w = obj.w;
  float h = obj.h;

  float r = (w < h? w: h) / 2;
  float rs = r * CIRATEFI_MAGNIFICATION_FACTOR;
  float re = r * CIRATEFI_MINIFICATION_FACTOR / pow(CIRATEFI_CIRCLE_DIVISOR, CIRATEFI_CIRCLES);

  float ri = r;
  for(; ri < rs; ri *= CIRATEFI_CIRCLE_DIVISOR);
  obj.circleRMax = ri;
  for(; ri > re; ri /= CIRATEFI_CIRCLE_DIVISOR) {
    ObjectDescription::CircleQuery query;
    query.r = ri;

    float rj = ri;
    query.pyrLvl = 0;

    float len = 2 * M_PI * rj;
    for(float alpha = 0; alpha < 2 * M_PI; alpha += 2 * M_PI / (len * CIRATEFI_CIRCLE_DENSITY)) {
      query.samplePoints.push_back(ObjectDescription::DeltaCoords{
        static_cast<int>(CIRATEFI_INTERPOLATION_SCALE * rj * cosf(alpha)) *
        CIRATEFI_INTERPOLATION_SCALE * (1 + img.data.cols) +
        static_cast<int>(CIRATEFI_INTERPOLATION_SCALE * rj * sinf(alpha))
      });
    }

    obj.circleQueries.push_back(query);
  }
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
  int cx = sx + w / 2;
  int cy = sy + h / 2;
  float r = (w < h? w: h) / 2;

  {
    float ri = r;
    for(int n = 0; n < CIRATEFI_CIRCLES_COARSE; ++n) {
      float len = 2 * M_PI * ri;
      float sum = 0;
      float count = 0;

      for(float alpha = 0; alpha < 2 * M_PI; alpha += 2 * M_PI / (len * CIRATEFI_CIRCLE_DENSITY)) {
        sum += interpolate(img, cx + ri * sinf(alpha), cy + ri * cosf(alpha), 255);
        count++;
      }
      result.circleValuesCoarse.push_back(sum / count);

      ri /= CIRATEFI_CIRCLE_DIVISOR_COARSE;
    }

    result.circleValuesCoarseMean = mean(result.circleValuesCoarse.begin(), result.circleValuesCoarse.end());
    for(auto &v: result.circleValuesCoarse) {
      v -= result.circleValuesCoarseMean;
    }
    result.circleValuesCoarseSquared = meanDotSquare(
        result.circleValuesCoarse.begin(), result.circleValuesCoarse.end(), 0.0
    );
  }

  float ri = r;
  for(int n = 0; n < CIRATEFI_CIRCLES; ++n) {
    float len = 2 * M_PI * ri;
    float sum = 0;
    float count = 0;

    for(float alpha = 0; alpha < 2 * M_PI; alpha += 2 * M_PI / (len * CIRATEFI_CIRCLE_DENSITY)) {
      sum += interpolate(img, cx + ri * sinf(alpha), cy + ri * cosf(alpha), 255);
      count++;
    }
    result.circleValues.push_back(sum / count);

    ri /= CIRATEFI_CIRCLE_DIVISOR;
  }

  result.circleValuesMean = mean(result.circleValues.begin(), result.circleValues.end());
  for(auto &v: result.circleValues) {
    v -= result.circleValuesMean;
  }
  result.circleValuesSquared = meanDotSquare(
      result.circleValues.begin(), result.circleValues.end(), 0.0
  );

  float alpha = 0;
  for(int n = 0; n < CIRATEFI_RADIALS; ++n) {
    float sum = 0;
    float count = 0;

    for(float ri = 0; ri < r; ++ri) {
      sum += interpolate(img, cx + ri * sinf(alpha), cy + ri * cosf(alpha), 128);
      count++;
    }
    alpha += 2 * M_PI / CIRATEFI_RADIALS;

    result.radialValues.push_back(sum / count);
  }

  result.radialValuesMean = mean(result.radialValues.begin(), result.radialValues.end());
  for(auto &v: result.radialValues) {
    v -= result.radialValuesMean;
  }
  result.radialValuesSquared = meanDotSquare(
      result.radialValues.begin(), result.radialValues.end(), 0.0
  );

  for(int n = 0; n < CIRATEFI_ORTHOGONALS; ++n) {
    float y = sy + h * 1.0 * n / (CIRATEFI_ORTHOGONALS - 1);

    float sum = 0;
    float count = 0;

    for(float x = sx; x < ex; ++x) {
      sum += interpolate(img, x, y, 128);
      count++;
    }

    result.orthogonalValues.push_back(sum / count);
  }

  for(int n = 0; n < CIRATEFI_PARALLELS; ++n) {
    float x = sx + w * 1.0 * n / (CIRATEFI_PARALLELS - 1);

    float sum = 0;
    float count = 0;

    for(float y = sy; y < ey; ++y) {
      sum += interpolate(img, x, y, 255);
      count++;
    }

    result.parallelValues.push_back(sum / count);
  }

  for(int y = sy; y < ey; ++y) {
    for(int x = sx; x < ex; ++x) {
      float sum = 0;
      float count = 0;

      for(float yy = y; yy < y + 1; yy += CIRATEFI_ALL_STEPS) {
        for(float xx = x; xx < x + 1; xx += CIRATEFI_ALL_STEPS) {
          sum += interpolate(img, xx, yy, 0);
          count++;
        }
      }

      result.allValues.push_back(sum / count);
    }
  }

  float allMean = mean(result.allValues.begin(), result.allValues.end());
  result.allValuesRange = meanDotSquare(
      result.allValues.begin(), result.allValues.end(), allMean
  ) / result.allValues.size();

  precomputeCoarseCircleQueries(img, result, 5);
  precomputePyramidCircleQueries(img, result, 20);
  precomputeCircleQueries(img, result);

  return result;
}

unsigned int querySum(unsigned char *center, const ObjectDescription::DeltaCoords *deltas, int n) {
  unsigned int sum_0 = 0;
  unsigned int sum_1 = 0;
  for(const ObjectDescription::DeltaCoords *e = deltas + (n >> 1 << 1); deltas != e; deltas += 2) {
    sum_0 += *(center + deltas[0].d);
    sum_1 += *(center + deltas[1].d);
  }
  unsigned int sum = sum_0 + sum_1;

  if(n & 1) {
    sum += *(center + deltas[0].d);
  }

  return sum;
}

struct CircleMatchQuality {
  float score;
  float scale;
  int startQuery;
  float beta, gamma;
};

CircleMatchQuality compareCirclesPyramidCoarse(const ObjectDescription &obj,
    std::vector<Image> &pyr, int x, int y,
    const ThresholdConfiguration &thresholds) {
  CircleMatchQuality result;
  std::vector<float> measured;

  for(const auto &query: obj.coarseCircleQueries) {
    float ri = query.r;

    if(x - ri < 0 || y - ri < 0 || x + ri >= pyr[0].cols || y + ri >= pyr[0].rows) {
      measured.push_back(-1e6);
    } else {
      int cx = (x * CIRATEFI_INTERPOLATION_SCALE >> query.pyrLvl);
      int cy = (y * CIRATEFI_INTERPOLATION_SCALE >> query.pyrLvl);
      Mat &data = pyr[query.pyrLvl].interpolationCache;
      const int center = cy * data.cols + cx;

      unsigned int sum = querySum(
          &data.at<unsigned char>(center),
          query.samplePoints.data(),
          query.samplePoints.size());

      measured.push_back(static_cast<float>(sum) / query.samplePoints.size());
    }
  }

  int bestIndex = 0;
  CorrelationCoefficient<float> bestCorrelation;
  bestCorrelation.corr = 0;
  bestCorrelation.beta = 0;
  bestCorrelation.gamma = 0;

  float measuredMeanSum = 0;
  for(size_t i = 0; i < CIRATEFI_CIRCLES_COARSE - 1; ++i) {
    measuredMeanSum += measured[i];
  }

  for(size_t i = 0; i < measured.size() - CIRATEFI_CIRCLES_COARSE + 1; ++i) {
    measuredMeanSum += measured[i + CIRATEFI_CIRCLES_COARSE - 1];

    auto corrCoeff = correlationCoefficientLeftConstRightMean(
      obj.circleValuesCoarse.begin(), obj.circleValuesCoarse.end(),
      obj.circleValuesCoarseMean, obj.circleValuesCoarseSquared,
      measured.begin() + i, measured.begin() + i + CIRATEFI_CIRCLES_COARSE,
      measuredMeanSum / CIRATEFI_CIRCLES_COARSE,
      thresholds);
    if(fabs(corrCoeff.corr) > bestCorrelation.corr) {
      bestCorrelation = corrCoeff;
      bestIndex = i;
    }

    measuredMeanSum -= measured[i];
  }

  result.score = fabs(bestCorrelation.corr);
  result.beta = bestCorrelation.beta;
  result.gamma = bestCorrelation.gamma;

  const float r = (obj.w < obj.h? obj.w: obj.h) / 2;
  result.scale = obj.coarseCircleRMax / (r * pow(CIRATEFI_CIRCLE_DIVISOR_COARSE, bestIndex));
  return result;
}

CircleMatchQuality compareCirclesPyramid(const ObjectDescription &obj,
    std::vector<Image> &pyr, int x, int y,
    const ThresholdConfiguration &thresholds) {
  CircleMatchQuality result;
  std::vector<float> measured;

  for(const auto &query: obj.pyramidCircleQueries) {
    float ri = query.r;

    if(x - ri < 0 || y - ri < 0 || x + ri >= pyr[0].cols || y + ri >= pyr[0].rows) {
      measured.push_back(-1e6);
    } else {
      int cx = (x * CIRATEFI_INTERPOLATION_SCALE >> query.pyrLvl);
      int cy = (y * CIRATEFI_INTERPOLATION_SCALE >> query.pyrLvl);
      Mat &data = pyr[query.pyrLvl].interpolationCache;
      const int center = cy * data.cols + cx;

      unsigned int sum = querySum(
          &data.at<unsigned char>(center),
          query.samplePoints.data(),
          query.samplePoints.size());

      measured.push_back(static_cast<float>(sum) / query.samplePoints.size());
    }
  }

  int bestIndex = 0;
  CorrelationCoefficient<float> bestCorrelation;
  bestCorrelation.corr = 0;
  bestCorrelation.beta = 0;
  bestCorrelation.gamma = 0;

  float measuredMeanSum = 0;
  for(size_t i = 0; i < CIRATEFI_CIRCLES - 1; ++i) {
    measuredMeanSum += measured[i];
  }

  for(size_t i = 0; i < measured.size() - CIRATEFI_CIRCLES + 1; ++i) {
    measuredMeanSum += measured[i + CIRATEFI_CIRCLES - 1];

    auto corrCoeff = correlationCoefficientLeftConstRightMean(
      obj.circleValues.begin(), obj.circleValues.end(),
      obj.circleValuesMean, obj.circleValuesSquared,
      measured.begin() + i, measured.begin() + i + CIRATEFI_CIRCLES,
      measuredMeanSum / CIRATEFI_CIRCLES,
      thresholds);
    if(fabs(corrCoeff.corr) > bestCorrelation.corr) {
      bestCorrelation = corrCoeff;
      bestIndex = i;
    }

    measuredMeanSum -= measured[i];
  }

  result.score = fabs(bestCorrelation.corr);
  result.beta = bestCorrelation.beta;
  result.gamma = bestCorrelation.gamma;
  result.startQuery = bestIndex;

  const float r = (obj.w < obj.h? obj.w: obj.h) / 2;
  result.scale = obj.pyramidCircleRMax / (r * pow(CIRATEFI_CIRCLE_DIVISOR, bestIndex));
  return result;
}

CircleMatchQuality compareCircles(const ObjectDescription &obj,
    Image &img, int x, int y, int startQuery, float scale,
    const ThresholdConfiguration &thresholds) {
  CircleMatchQuality result;
  std::array<float, CIRATEFI_CIRCLES> measured;

  auto query = obj.circleQueries.begin() + startQuery;
  for(int i = 0; i < CIRATEFI_CIRCLES; ++i, ++query) {
    float ri = query->r;

    if(x - ri < 0 || y - ri < 0 || x + ri >= img.cols || y + ri >= img.rows) {
      measured[i] = -1e6;
    } else {
      int cx = x * CIRATEFI_INTERPOLATION_SCALE;
      int cy = y * CIRATEFI_INTERPOLATION_SCALE;
      Mat &data = img.interpolationCache;
      const int center = cy * data.cols + cx;

      unsigned int sum = querySum(
          &data.at<unsigned char>(center),
          query->samplePoints.data(),
          query->samplePoints.size());

      measured[i] = static_cast<float>(sum) / query->samplePoints.size();
    }
  }

  float bestCorrelation = 0.0;
  float corrCoeff = correlationCoefficientLeftConst(
    obj.circleValues.begin(), obj.circleValues.end(),
    obj.circleValuesMean, obj.circleValuesSquared,
    measured.begin(), measured.end(),
    thresholds).corr;
  if(fabs(corrCoeff) > bestCorrelation) {
    bestCorrelation = fabs(corrCoeff);
  }

  result.score = bestCorrelation;
  result.scale = scale;
  return result;
}

CircleMatchQuality compareCirclesShifted(const ObjectDescription &obj,
    Image &img, int x, int y, float scale,
    const ThresholdConfiguration &thresholds) {
  CircleMatchQuality result;

  float w = obj.w;
  float h = obj.h;
  int cx = x;
  int cy = y;
  float r = (w < h? w: h) / 2;
  float rs = r * scale / pow(CIRATEFI_CIRCLE_SHIFT, 5);
  float re = r * scale * pow(CIRATEFI_CIRCLE_SHIFT, 5);

  int bestRadius = 0;
  float bestCorrelation = 0.0;

  for(float rj = rs; rj < re; rj *= CIRATEFI_CIRCLE_SHIFT) {
    float ri = rj;
    std::vector<float> measured;

    for(int n = 0; n < CIRATEFI_CIRCLES; ++n) {
      float len = 2 * M_PI * ri;
      float sum = 0;
      float count = 0;

      for(float alpha = 0; alpha < 2 * M_PI; alpha += 2 * M_PI / (len * CIRATEFI_CIRCLE_DENSITY)) {
        sum += interpolate(img, cx + ri * sinf(alpha), cy + ri * cosf(alpha), 64);
        count++;
      }

      measured.push_back(sum / count);

      ri /= CIRATEFI_CIRCLE_DIVISOR;
    }

    float corrCoeff = correlationCoefficientLeftConst(
      obj.circleValues.begin(), obj.circleValues.end(),
      obj.circleValuesMean, obj.circleValuesSquared,
      measured.begin(), measured.begin() + CIRATEFI_CIRCLES,
      thresholds).corr;
    if(fabs(corrCoeff) > bestCorrelation) {
      bestCorrelation = fabs(corrCoeff);
      bestRadius = rj;
    }
  }

  result.score = bestCorrelation;
  result.scale = bestRadius / r;
  return result;
}

struct RadialMatchQuality {
  float score;
  float angle;
};

RadialMatchQuality compareRadials(const ObjectDescription &obj,
    Image &img, int x, int y, float scale,
    const ThresholdConfiguration &thresholds) {
  RadialMatchQuality result;
  std::vector<float> measured;

  float w = obj.w;
  float h = obj.h;
  int cx = x;
  int cy = y;
  float r = scale * (w < h? w: h) / 2;

  const int center = static_cast<int>(cy * CIRATEFI_INTERPOLATION_SCALE) *
      img.interpolationCache.cols +
      static_cast<int>(cx * CIRATEFI_INTERPOLATION_SCALE);
  float alpha = 0;
  for(int n = 0; n < CIRATEFI_RADIALS; ++n) {
    int sum = 0;
    float count = 0;

    for(float ri = 0; ri < r; ri += 1 / CIRATEFI_RADIAL_DENSITY) {
      sum += img.interpolationCache.at<unsigned char>(
          center +
          static_cast<int>(ri * cosf(alpha) * CIRATEFI_INTERPOLATION_SCALE) *
          img.interpolationCache.cols +
          static_cast<int>(ri * sinf(alpha) * CIRATEFI_INTERPOLATION_SCALE));
      // sum += img.interpolationCache.at<unsigned char>(
      //     static_cast<int>((cy + ri * cosf(alpha)) * CIRATEFI_INTERPOLATION_SCALE) *
      //     img.interpolationCache.cols +
      //     static_cast<int>((cx + ri * sinf(alpha)) * CIRATEFI_INTERPOLATION_SCALE));
      // sum += interpolate(img, cx + ri * sinf(alpha), cy + ri * cosf(alpha), 128);
      count++;
    }
    alpha += 2 * M_PI / CIRATEFI_RADIALS;

    measured.push_back(static_cast<float>(sum) / count);
  }

  int bestIndex = 0;
  float bestCorrelation = 0.0;

  float measuredMeanSum = 0;
  for(int n = 0; n < CIRATEFI_RADIALS; ++n) {
    measured.push_back(measured[n]);
    measuredMeanSum += measured[n];
  }
  const float measuredMean = measuredMeanSum / CIRATEFI_RADIALS;
  for(int n = 0; n < CIRATEFI_RADIALS * 2; ++n) {
    measured[n] -= measuredMean;
  }
  float measuredSquared = 0;
  for(int n = 0; n < CIRATEFI_RADIALS; ++n) {
    measuredSquared += measured[n] * measured[n];
  }

  for(size_t i = 0; i < measured.size() - CIRATEFI_RADIALS; ++i) {
    float corrCoeff = correlationCoefficientLeftConstRightConst(
      obj.radialValues.begin(), obj.radialValues.end(),
      obj.radialValuesMean, obj.radialValuesSquared,
      measured.begin() + i, measured.begin() + i + CIRATEFI_RADIALS,
      measuredMean, measuredSquared,
      thresholds).corr;
    if(fabs(corrCoeff) > bestCorrelation) {
      bestCorrelation = fabs(corrCoeff);
      bestIndex = i;
    }
  }

  result.score = bestCorrelation;
  result.angle = bestIndex * 2 * M_PI / CIRATEFI_RADIALS;
  return result;
}

struct OrthogonalMatchQuality {
  float yCorrection;
  float score;
};

OrthogonalMatchQuality compareOrthogonals(const ObjectDescription &obj,
    Image &img, int x, int y, float scale, float angle,
    const ThresholdConfiguration &thresholds) {
  OrthogonalMatchQuality result;

  float bestYCorrection = -1;
  float bestCorrelation = 0.0;

  for(float yCorrection = 0.8; yCorrection < 1.2; yCorrection += 0.01) {
    std::vector<float> measured;

    float wx = scale * sinf(angle + 0.5 * M_PI);
    float wy = scale * cosf(angle + 0.5 * M_PI);
    float hx = scale * yCorrection * sinf(angle);
    float hy = scale * yCorrection * cosf(angle);

    int sx = x - (obj.w / 2.0) * wx - (obj.h / 2.0) * hx;
    int sy = y - (obj.w / 2.0) * wy - (obj.h / 2.0) * hy;

    for(int n = 0; n < CIRATEFI_ORTHOGONALS; ++n) {
      float sum = 0;
      float count = 0;

      float ly = obj.h * 1.0 * n / (CIRATEFI_ORTHOGONALS - 1);
      for(int lx = 0; lx < obj.w; ++lx) {
        float ix = sx + lx * wx + ly * hx;
        float iy = sy + lx * wy + ly * hy;

        sum += interpolate(img, ix, iy, 128);
        count++;
      }

      measured.push_back(sum / count);
    }

    float corrCoeff = correlationCoefficient(
      obj.orthogonalValues.begin(), obj.orthogonalValues.end(),
      measured.begin(), measured.end(),
      thresholds).corr;
    if(fabs(corrCoeff) > bestCorrelation) {
      bestCorrelation = fabs(corrCoeff);
      bestYCorrection = yCorrection;
    }
  }

  result.score = bestCorrelation;
  result.yCorrection = bestYCorrection;
  return result;
}

struct ParallelMatchQuality {
  float xCorrection;
  float score;
};

ParallelMatchQuality compareParallels(const ObjectDescription &obj,
    Image &img, int x, int y, float scale, float angle, float yCorrection,
    const ThresholdConfiguration &thresholds) {
  ParallelMatchQuality result;

  float bestXCorrection = -1;
  float bestCorrelation = 0.0;

  for(float xCorrection = 0.7; xCorrection < 1.3; xCorrection += 0.01) {
    std::vector<float> measured;

    float wx = scale * xCorrection * sinf(angle + 0.5 * M_PI);
    float wy = scale * xCorrection * cosf(angle + 0.5 * M_PI);
    float hx = scale * yCorrection * sinf(angle);
    float hy = scale * yCorrection * cosf(angle);

    int sx = x - (obj.w / 2.0) * wx - (obj.h / 2.0) * hx;
    int sy = y - (obj.w / 2.0) * wy - (obj.h / 2.0) * hy;

    for(int n = 0; n < CIRATEFI_PARALLELS; ++n) {
      float sum = 0;
      float count = 0;

      float lx = obj.w * 1.0 * n / (CIRATEFI_PARALLELS - 1);
      for(int ly = 0; ly < obj.h; ++ly) {
        float ix = sx + lx * wx + ly * hx;
        float iy = sy + lx * wy + ly * hy;

        sum += interpolate(img, ix, iy, 128);
        count++;
      }

      measured.push_back(sum / count);
    }

    float corrCoeff = correlationCoefficient(
      obj.parallelValues.begin(), obj.parallelValues.end(),
      measured.begin(), measured.end(),
      thresholds).corr;
    if(fabs(corrCoeff) > bestCorrelation) {
      bestCorrelation = fabs(corrCoeff);
      bestXCorrection = xCorrection;
    }
  }

  result.score = bestCorrelation;
  result.xCorrection = bestXCorrection;
  return result;
}

struct AllMatchQuality {
  float score;
  float beta, gamma;
  float x, y, wx, wy, hx, hy;
};

AllMatchQuality compareAllMatrix(const ObjectDescription &obj,
    Image &img, int x, int y, float wx, float wy, float hx, float hy,
    const ThresholdConfiguration &thresholds) {
  AllMatchQuality result;
  result.score = 0;

  std::vector<float> measured;
  float bestCorrelation = 0.0;

  int sx = x - (obj.w / 2.0) * wx - (obj.h / 2.0) * hx;
  int sy = y - (obj.w / 2.0) * wy - (obj.h / 2.0) * hy;

  for(int ly = 0; ly < obj.h; ++ly) {
    for(int lx = 0; lx < obj.w; ++lx) {
      float sum = 0;
      int count = 0;

      for(float yy = ly; yy < ly + 1; yy += CIRATEFI_ALL_STEPS) {
        for(float xx = lx; xx < lx + 1; xx += CIRATEFI_ALL_STEPS) {
          float ix = sx + xx * wx + yy * hx;
          float iy = sy + xx * wy + yy * hy;

          sum += interpolate(img, ix, iy, 255);
          count++;
        }
      }

      measured.push_back(sum / count);
    }
  }

  auto corrCoeff = correlationCoefficient(
    obj.allValues.begin(), obj.allValues.end(),
    measured.begin(), measured.end(),
    thresholds);
  if(fabs(corrCoeff.corr) > bestCorrelation) {
    bestCorrelation = fabs(corrCoeff.corr);
    result.beta = corrCoeff.beta;
    result.gamma = corrCoeff.gamma;
    result.score = bestCorrelation;
    result.wx = wx;
    result.wy = wy;
    result.hx = hx;
    result.hy = hy;
    result.x = x;
    result.y = y;
  }

  return result;
}

AllMatchQuality compareAll(const ObjectDescription &obj,
    Image &img, int x, int y, float scale, float angle,
    float yCorrection, float xCorrection,
    const ThresholdConfiguration &thresholds) {
  // TODO: Allow optimization of object center

  float wx = scale * xCorrection * sinf(angle + 0.5 * M_PI);
  float wy = scale * xCorrection * cosf(angle + 0.5 * M_PI);
  float hx = scale * yCorrection * sinf(angle);
  float hy = scale * yCorrection * cosf(angle);

  auto result = compareAllMatrix(obj, img, x, y, wx, wy, hx, hy,
      thresholds);
  if(result.score < thresholds.allEarlyThreshold) {
    return result;
  }

  for(int n = 0; n < CIRATEFI_FINAL_WIGGLES; ++n) {
    float nwx = wx + sinf(rand()) * CIRATEFI_FINAL_WIGGLE_STEP;
    float nwy = wy + sinf(rand()) * CIRATEFI_FINAL_WIGGLE_STEP;
    float nhx = hx + sinf(rand()) * CIRATEFI_FINAL_WIGGLE_STEP;
    float nhy = hy + sinf(rand()) * CIRATEFI_FINAL_WIGGLE_STEP;

    auto nresult = compareAllMatrix(obj, img, x, y, nwx, nwy, nhx, nhy,
        thresholds);
    if(nresult.score > result.score) {
      result = nresult;
      wx = nwx; wy = nwy; hx = nhx; hy = nhy;
    }
  }

  return result;
}

void drawMatch(const ObjectDescription &obj,
    Mat &img, const AllMatchQuality &match, int, int g, int b) {
  int sx = match.x - (obj.w / 2.0) * match.wx - (obj.h / 2.0) * match.hx;
  int sy = match.y - (obj.w / 2.0) * match.wy - (obj.h / 2.0) * match.hy;

  for(int ly = 0; ly < obj.h; ++ly) {
    for(int lx = 0; lx < obj.w; ++lx) {
      float ix = sx + lx * match.wx + ly * match.hx;
      float iy = sy + lx * match.wy + ly * match.hy;

      if(iy > 0 && ix > 0 && iy < img.rows && ix < img.cols) {
        img.at<rgb>(iy, ix).g = g;
        img.at<rgb>(iy, ix).b = b;
      }
    }
  }
}

std::vector<Image> buildImagePyramid(Image img) {
  std::vector<Image> ret;
  ret.push_back(img);

  for(int n = 0; n < CIRATEFI_PYRAMID_LEVELS; ++n) {
    Image &cur = ret.back();
    Mat next(cur.rows / 2, cur.cols / 2, CV_8UC3);

    for(int y = 0; y < next.rows; ++y) {
      for(int x = 0; x < next.cols; ++x) {
        unsigned char val = (
          static_cast<unsigned int>(cur.data.at<rgb>(y * 2, x * 2).r) +
          static_cast<unsigned int>(cur.data.at<rgb>(y * 2, x * 2 + 1).r) +
          static_cast<unsigned int>(cur.data.at<rgb>(y * 2 + 1, x * 2).r) +
          static_cast<unsigned int>(cur.data.at<rgb>(y * 2 + 1, x * 2 + 1).r)
        ) / 4;
        next.at<rgb>(y, x) = rgb{ val, val, val };
      }
    }

    ret.push_back(Image(next));
  }

  return ret;
}

template<class F> Mat buildExtremumImage(Mat &img, int range, const F &f) {
  Mat ret = img.clone();

  for(int i = 0; i < range; ++i) {
    Mat next = ret.clone();
    for(int y = 0; y < ret.rows; ++y) {
      for(int x = 0; x < ret.cols; ++x) {
        unsigned char ext = ret.at<rgb>(y, x).r;

        // if(x > 0 && y > 0)                   ext = f(ext, ret.at<rgb>(y - 1, x - 1).r);
        if(x > 0)                            ext = f(ext, ret.at<rgb>(y,     x - 1).r);
        // if(x > 0 && y < img.rows - 1)        ext = f(ext, ret.at<rgb>(y + 1, x - 1).r);
        if(y > 0)                            ext = f(ext, ret.at<rgb>(y - 1, x).r);
        if(y < img.rows - 1)                 ext = f(ext, ret.at<rgb>(y + 1, x).r);
        //if(x < img.cols && y > 0)            ext = f(ext, ret.at<rgb>(y - 1, x + 1).r);
        if(x < img.cols)                     ext = f(ext, ret.at<rgb>(y,     x + 1).r);
        // if(x < img.cols && y < img.rows - 1) ext = f(ext, ret.at<rgb>(y + 1, x + 1).r);

        next.at<rgb>(y, x).r = ext;
      }
    }

    ret = next;
  }

  return ret;
}

class CiratefiTracker {
  public:
    CiratefiTracker(int width, int height): cols(width), rows(height) { }

    void setTemplate(Mat &reference, int sx, int sy, int ex, int ey) {
      Image referenceImage(reference);
      templ = measureObject(referenceImage, sx, sy, ex, ey);
    }

    typedef AllMatchQuality ObjectPosition;

    ObjectPosition handleFrame(Mat &query) {
      ObjectPosition newPos;
      newPos.score = -1;

      Image queryImage(query);
      auto queryPyramid = buildImagePyramid(queryImage);

      if(lastObj.w != 0 && newPos.score <= 0) {
        newPos = searchObject(queryPyramid, lastObj, FAST_TRACKING_THRESHOLDS,
            lastPos.x - 20, lastPos.y - 20, lastPos.x + 20, lastPos.y + 20);
        if(newPos.score > 0) {
          remeasureObject(queryPyramid, lastObj, newPos);
          showResult(query, newPos, 0, 255, 0);
        }
      }

      if(lastObj.w != 0 && newPos.score <= 0) {
        newPos = searchObject(queryPyramid, lastObj, FAST_TRACKING_THRESHOLDS, 0, 0, cols, rows);
        if(newPos.score > 0) {
          remeasureObject(queryPyramid, lastObj, newPos);
          showResult(query, newPos, 0, 255, 0);
        }
      }

      if(newPos.score <= 0) {
        newPos = searchObject(queryPyramid, templ, SEARCH_THRESHOLDS,
            lastPos.x - 80, lastPos.y - 80, lastPos.x + 80, lastPos.y + 80);
        if(newPos.score > 0) {
          remeasureObject(queryPyramid, templ, newPos);
          showResult(query, newPos, 0, 190, 190);
        }
      }

      if(newPos.score <= 0) {
        newPos = searchObject(queryPyramid, templ, SEARCH_THRESHOLDS, 0, 0, cols, rows);
        if(newPos.score > 0) {
          remeasureObject(queryPyramid, templ, newPos);
          showResult(query, newPos, 0, 190, 190);
        }
      }

      if(lastObj.w != 0 && newPos.score <= 0) {
        newPos = searchObject(queryPyramid, lastObj, TRACKING_THRESHOLDS,
            lastPos.x - 20, lastPos.y - 20, lastPos.x + 20, lastPos.y + 20);
        if(newPos.score > 0) {
          remeasureObject(queryPyramid, lastObj, newPos);
          showResult(query, newPos, 0, 0, 255);
        }
      }

      if(lastObj.w != 0 && newPos.score <= 0) {
        newPos = searchObject(queryPyramid, lastObj, TRACKING_THRESHOLDS, 0, 0, cols, rows);
        if(newPos.score > 0) {
          remeasureObject(queryPyramid, lastObj, newPos);
          showResult(query, newPos, 0, 0, 255);
        }
      }

      if(newPos.score <= 0) {
        lastObj = ObjectDescription();
      }

      imshow("Display window 2", query);

      for(int i = 0; i < 5; ++i) {
        if( waitKey(1) >= 0 ) {
          throw 0;
        }
      }

      lastPos = newPos;
      return lastPos;
    }

    void remeasureObject(std::vector<Image> &queryPyramid, ObjectDescription outline, ObjectPosition pos) {
      lastObj = measureObject(queryPyramid[0],
          pos.x - 0.5 * outline.w * pos.wx,
          pos.y - 0.5 * outline.h * pos.hy,
          pos.x + 0.5 * outline.w * pos.wx,
          pos.y + 0.5 * outline.h * pos.hy);
    }

    void showResult(Mat &query, const ObjectPosition &pos, int r, int g, int b) {
      drawMatch(lastObj, query, pos, r, g, b);
    }

    ObjectPosition searchObject(std::vector<Image> &queryPyramid, const ObjectDescription &obj,
        const ThresholdConfiguration &thresholds, int sx, int sy, int ex, int ey) {
      Image &queryImage = queryPyramid[0];
      Mat &query = queryImage.data;

      auto minImage = buildExtremumImage(queryPyramid[2].data, 16,
          static_cast<const unsigned char &(*)(const unsigned char &, const unsigned char&)>(std::min));
      auto maxImage = buildExtremumImage(queryPyramid[2].data, 16,
          static_cast<const unsigned char &(*)(const unsigned char &, const unsigned char&)>(std::max));
      Mat hopeless(query.rows + 1, query.cols, CV_8UC1, 0.0);

      std::vector<ObjectPosition> hits;

      sx = sx < 0? 0: sx;
      sy = sy < 0? 0: sy;
      ex = ex > query.cols? query.cols: ex;
      ey = ey > query.rows? query.cols: ey;

      for(int y = sy; y < ey; ++y) {
        cout << y << endl;
        for(int x = sx; x < ex; ++x) {
          if(hopeless.at<unsigned char>(y, x)) continue;

          float range = maxImage.at<rgb>(y / 4, x / 4).r - minImage.at<rgb>(y / 4, x / 4).r;
          if(range * range < obj.allValuesRange) continue;

          auto circleMatchCoarse = compareCirclesPyramidCoarse(obj, queryPyramid, x, y, thresholds);
          if(circleMatchCoarse.score < 0.1) {
            x++;
            hopeless.at<unsigned char>(y + 1, x - 1) = 1;
            hopeless.at<unsigned char>(y + 1, x) = 1;
            hopeless.at<unsigned char>(y + 1, x + 1) = 1;
            continue;
          }

          if(circleMatchCoarse.score < thresholds.circleThresholdCoarse) continue;

          // query.at<rgb>(y, x).g = 64;

          auto circleMatchPyramid = compareCirclesPyramid(obj, queryPyramid, x, y, thresholds);
          if(circleMatchPyramid.score < thresholds.circleThresholdPyramid) continue;

          //query.at<rgb>(y, x).g = 64;

          auto radialMatch = compareRadials(obj, queryImage, x, y, circleMatchPyramid.scale, thresholds);
          if(radialMatch.score < thresholds.radialThreshold) continue;

          //query.at<rgb>(y, x).g = 80;

          auto circleMatch = compareCircles(obj, queryImage, x, y,
              circleMatchPyramid.startQuery, circleMatchPyramid.scale, thresholds);
          // cout << circleMatch.score << endl;
          if(circleMatch.score < thresholds.circleThreshold) continue;

          //query.at<rgb>(y, x).g = 192;
          auto circleMatchShifted = compareCirclesShifted(obj, queryImage, x, y, circleMatch.scale, thresholds);
          if(circleMatchShifted.score < thresholds.circleThresholdShifted) continue;

          //query.at<rgb>(y, x).g = 128;

          auto orthogonalMatch = compareOrthogonals(obj, queryImage, x, y,
              circleMatchShifted.scale, radialMatch.angle, thresholds);
          if(orthogonalMatch.score < thresholds.orthogonalThreshold) continue;

          query.at<rgb>(y, x).b = 128;

          //query.at<rgb>(y, x).b = 64;

          auto parallelMatch = compareParallels(obj, queryImage, x, y,
              circleMatchShifted.scale, radialMatch.angle, orthogonalMatch.yCorrection, thresholds);
          if(parallelMatch.score < thresholds.parallelThreshold) continue;

          query.at<rgb>(y, x).b = 255;

          //query.at<rgb>(y, x).b = 128;

          auto allMatch = compareAll(obj, queryImage, x, y,
              circleMatchShifted.scale, radialMatch.angle,
              orthogonalMatch.yCorrection, parallelMatch.xCorrection,
              thresholds);
          if(allMatch.score < thresholds.allThreshold) continue;

          query.at<rgb>(y, x).g = 255;
          //query.at<rgb>(y, x).g = 255;

          cout
            << "x,y: " << x << "," << y
            << " C: " << circleMatch.score
            << " C2: " << circleMatchShifted.score
            << " S: " << circleMatch.scale
            << " S2: " << circleMatchShifted.scale
            << " A: " << radialMatch.angle
            << " y: " << orthogonalMatch.yCorrection
            << " x: " << parallelMatch.xCorrection
            << " R: " << radialMatch.score
            << " O: " << orthogonalMatch.score
            << " P: " << parallelMatch.score
            << " A: " << allMatch.score
            << " β: " << allMatch.beta
            << " γ: " << allMatch.gamma
            << " P: " << circleMatchPyramid.score
            << " cPβ: " << circleMatchPyramid.beta
            << " cPγ: " << circleMatchPyramid.gamma
            << " CC: " << circleMatchCoarse.score
            // << " P: " << circleMatchPyramid2.score
            << endl;

          hits.push_back(allMatch);
        }
      }

      ObjectPosition best;
      best.score = 0;
      for(const auto &hit: hits) {
        if(hit.score > best.score) {
          best = hit;
        }
      }

      return best;
    }
  
  private:
    ObjectDescription templ;

    ObjectPosition lastPos;
    ObjectDescription lastObj;

    int cols, rows;
};

int main(int, char**)
{
    namedWindow("Display window 2", WINDOW_AUTOSIZE);

    Mat reference = imread("svo.0120.png", IMREAD_COLOR );
    GaussianBlur(reference, reference, cv::Size{0,0}, 1, 1);
    if(reference.empty()) {
        cout <<  "Could not load reference image" << endl ;
        return -1;
    }

    int sx = 370;
    int ex = 415;
    int sy = 130;
    int ey = 175;

    rectangle(reference,
              Point(sx - 2, sy - 2),
              Point(ex + 2, ey + 2),
              Scalar(0, 255, 0),
              1,
              8);

    CiratefiTracker tracker(reference.cols, reference.rows);
    tracker.setTemplate(reference, sx, sy, ex, ey);

    for(int i = 7; i < 199; ++i) {
      ostringstream filename;
      filename << "svo." << setw(4) << setfill('0') << i << ".png";

      Mat query = imread(filename.str(), IMREAD_COLOR );
      GaussianBlur(query, query, cv::Size{0,0}, 1, 1);
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
