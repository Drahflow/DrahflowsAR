#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

struct __attribute__((__packed__)) rgb {
  unsigned char b, g, r;
};

struct ObjectDescription {
  float w, h;
  std::vector<float> circleValues;
  std::vector<float> radialValues;
  std::vector<float> orthogonalValues;
  std::vector<float> parallelValues;
  std::vector<float> allValues;
};

const int CIRATEFI_CIRCLES = 15;
const int CIRATEFI_RADIALS = 256;
const int CIRATEFI_ORTHOGONALS = 16;
const int CIRATEFI_PARALLELS = 16;
const float CIRATEFI_CIRCLE_DIVISOR = 1.07;
const float CIRATEFI_MAGNIFICATION_FACTOR = 3;
const float CIRATEFI_MINIFICATION_FACTOR = 0.3;
const float CIRATEFI_RADIAL_DENSITY = 0.5;
const float CIRATEFI_CONTRAST_THRESHOLD = 1500;
const float CIRATEFI_CIRCLE_THRESHOLD = 0.95;
const float CIRATEFI_RADIAL_THRESHOLD = 0.75;
const float CIRATEFI_ORTHOGONAL_THRESHOLD = 0.40;
const float CIRATEFI_PARALLEL_THRESHOLD = 0.65;
const float CIRATEFI_ALL_THRESHOLD = 0.25;

struct InterpolatedValue {
  float v;
  bool exists;
};

InterpolatedValue interpolate(Mat &img, float x, float y, unsigned char draw) {
  int lx = x;
  int ly = y;
  int hx = lx + 1;
  int hy = ly + 1;

  float weight_x = x - lx;
  float weight_y = y - ly;

  if(lx < 0 || ly < 0 || hx >= img.cols || hy >= img.rows)
    return { 0, false, };

  float v00 = img.at<rgb>(ly, lx).r;
  float v10 = img.at<rgb>(hy, lx).r;
  float v01 = img.at<rgb>(ly, hx).r;
  float v11 = img.at<rgb>(hy, hx).r;

  float lcol = weight_y * v00 + (1 - weight_y) * v01;
  float hcol = weight_y * v10 + (1 - weight_y) * v11;

  img.at<rgb>(ly, lx).b = std::max(draw, img.at<rgb>(ly, lx).b);

  return {
    weight_x * hcol + (1 - weight_x) * lcol,
    true,
  };
}

ObjectDescription measureObject(Mat &img, int sx, int sy, int ex, int ey) {
  ObjectDescription result;

  float w = ex - sx; result.w = w;
  float h = ey - sy; result.h = h;
  int cx = sx + w / 2;
  int cy = sy + h / 2;
  float r = (w < h? w: h) / 2;

  float ri = r;
  for(int n = 0; n < CIRATEFI_CIRCLES; ++n) {
    float len = 2 * M_PI * ri;
    float sum = 0;
    float count = 0;

    for(float alpha = 0; alpha < 2 * M_PI; alpha += 2 * M_PI / (len * CIRATEFI_RADIAL_DENSITY)) {
      auto v = interpolate(img, cx + ri * sin(alpha), cy + ri * cos(alpha), 255);
      if(v.exists) {
        sum += v.v;
        count++;
      }
    }
    ri /= CIRATEFI_CIRCLE_DIVISOR;

    result.circleValues.push_back(sum / count);
  }

  float alpha = 0;
  for(int n = 0; n < CIRATEFI_RADIALS; ++n) {
    float sum = 0;
    float count = 0;

    for(float ri = 0; ri < r; ++ri) {
      auto v = interpolate(img, cx + ri * sin(alpha), cy + ri * cos(alpha), 128);
      if(v.exists) {
        sum += v.v;
        count++;
      }
    }
    alpha += 2 * M_PI / CIRATEFI_RADIALS;

    result.radialValues.push_back(sum / count);
  }

  for(int n = 0; n < CIRATEFI_ORTHOGONALS; ++n) {
    float y = sy + h * 1.0 * n / (CIRATEFI_ORTHOGONALS - 1);

    float sum = 0;
    float count = 0;

    for(float x = sx; x < ex; ++x) {
      auto v = interpolate(img, x, y, 128);
      if(v.exists) {
        sum += v.v;
        count++;
      }
    }

    result.orthogonalValues.push_back(sum / count);
  }

  for(int n = 0; n < CIRATEFI_PARALLELS; ++n) {
    float x = sx + w * 1.0 * n / (CIRATEFI_PARALLELS - 1);

    float sum = 0;
    float count = 0;

    for(float y = sy; y < ey; ++y) {
      auto v = interpolate(img, x, y, 255);
      if(v.exists) {
        sum += v.v;
        count++;
      }
    }

    result.parallelValues.push_back(sum / count);
  }

  for(int y = sy; y < ey; ++y) {
    for(int x = sx; x < ex; ++x) {
      auto v = interpolate(img, x, y, 0);

      result.allValues.push_back(v.v);
    }
  }

  return result;
}

template<typename I> typename I::value_type mean(
    const I &is, const I &ie) {
  typename I::value_type sum = 0;
  int count = 0;

  for(auto i = is; i != ie; ++i) {
    sum += *i;
    count++;
  }

  return sum / count;
}
  
template<typename I, typename IT, typename J, typename JT> typename I::value_type meanDotProduct(
    const I &is, const I &ie, const IT &im, const J &js, const J &je, const JT &jm) {
  typename I::value_type p = 0;

  auto i = is;
  auto j = js;

  while(i != ie && j != je) {
    p += (*i - im) * (*j - jm);
    i++; j++;
  }

  return p;
}

template<typename I, typename J> typename I::value_type correlationCoefficient(
    const I &is, const I &ie, const J &js, const J &je, int n) {
  const typename I::value_type xm = mean(is, ie);
  const typename J::value_type ym = mean(js, je);

  const typename I::value_type xx = meanDotProduct(is, ie, xm, is, ie, xm);
  const typename J::value_type yy = meanDotProduct(js, je, ym, js, je, ym);
  const typename I::value_type xy = meanDotProduct(is, ie, xm, js, je, ym);

  if(n * yy < CIRATEFI_CONTRAST_THRESHOLD || n * xx < CIRATEFI_CONTRAST_THRESHOLD) {
    return 0;
  }

  float result = xy / sqrt(xx * yy);

  return result;
}

struct CircleMatchQuality {
  float score;
  float scale;
};

CircleMatchQuality compareCircles(const ObjectDescription &obj,
    Mat &img, int x, int y) {
  CircleMatchQuality result;
  std::vector<float> measured;

  float w = obj.w;
  float h = obj.h;
  int cx = x;
  int cy = y;
  float r = (w < h? w: h) / 2;
  float rs = r * CIRATEFI_MAGNIFICATION_FACTOR;
  float re = r * CIRATEFI_MINIFICATION_FACTOR / pow(CIRATEFI_CIRCLE_DIVISOR, CIRATEFI_CIRCLES);

  float ri = r;
  for(; ri < rs; ri *= CIRATEFI_CIRCLE_DIVISOR);
  const float rMax = ri;
  for(; ri > re; ri /= CIRATEFI_CIRCLE_DIVISOR) {
    float len = 2 * M_PI * ri;
    float sum = 0;
    float count = 0;

    for(float alpha = 0; alpha < 2 * M_PI; alpha += 2 * M_PI / (len * CIRATEFI_RADIAL_DENSITY)) {
      auto v = interpolate(img, cx + ri * sin(alpha), cy + ri * cos(alpha), 64);
      if(v.exists) {
        sum += v.v;
        count++;
      }
    }

    measured.push_back(sum / count);
  }

  int bestIndex = 0;
  float bestCorrelation = 0.0;
  for(size_t i = 0; i < measured.size() - CIRATEFI_CIRCLES; ++i) {
    float corrCoeff = correlationCoefficient(
      obj.circleValues.begin(), obj.circleValues.end(),
      measured.begin() + i, measured.begin() + i + CIRATEFI_CIRCLES,
      CIRATEFI_CIRCLES);
    if(fabs(corrCoeff) > bestCorrelation) {
      bestCorrelation = fabs(corrCoeff);
      bestIndex = i;
    }
  }

  result.score = bestCorrelation;
  result.scale = rMax / (r * pow(CIRATEFI_CIRCLE_DIVISOR, bestIndex));
  return result;
}

struct RadialMatchQuality {
  float score;
  float angle;
};

RadialMatchQuality compareRadials(const ObjectDescription &obj,
    Mat &img, int x, int y, float scale) {
  RadialMatchQuality result;
  std::vector<float> measured;

  float w = obj.w;
  float h = obj.h;
  int cx = x;
  int cy = y;
  float r = scale * (w < h? w: h) / 2;

  float alpha = 0;
  for(int n = 0; n < CIRATEFI_RADIALS; ++n) {
    float sum = 0;
    float count = 0;

    for(float ri = 0; ri < r; ++ri) {
      auto v = interpolate(img, cx + ri * sin(alpha), cy + ri * cos(alpha), 128);
      if(v.exists) {
        sum += v.v;
        count++;
      }
    }
    alpha += 2 * M_PI / CIRATEFI_RADIALS;

    measured.push_back(sum / count);
  }

  int bestIndex = 0;
  float bestCorrelation = 0.0;

  for(int n = 0; n < CIRATEFI_RADIALS; ++n) {
    measured.push_back(measured[n]);
  }

  for(size_t i = 0; i < measured.size() - CIRATEFI_RADIALS; ++i) {
    float corrCoeff = correlationCoefficient(
      obj.radialValues.begin(), obj.radialValues.end(),
      measured.begin() + i, measured.begin() + i + CIRATEFI_RADIALS,
      CIRATEFI_RADIALS);
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
    Mat &img, int x, int y, float scale, float angle) {
  OrthogonalMatchQuality result;

  float bestYCorrection = -1;
  float bestCorrelation = 0.0;

  for(float yCorrection = 0.8; yCorrection < 1.2; yCorrection += 0.01) {
    std::vector<float> measured;

    float wx = scale * sin(angle + 0.5 * M_PI);
    float wy = scale * cos(angle + 0.5 * M_PI);
    float hx = scale * yCorrection * sin(angle);
    float hy = scale * yCorrection * cos(angle);

    int sx = x - (obj.w / 2.0) * wx - (obj.h / 2.0) * hx;
    int sy = y - (obj.w / 2.0) * wy - (obj.h / 2.0) * hy;

    for(int n = 0; n < CIRATEFI_ORTHOGONALS; ++n) {
      float sum = 0;
      float count = 0;

      float ly = obj.h * 1.0 * n / (CIRATEFI_ORTHOGONALS - 1);
      for(int lx = 0; lx < obj.w; ++lx) {
        float ix = sx + lx * wx + ly * hx;
        float iy = sy + lx * wy + ly * hy;

        auto v = interpolate(img, ix, iy, 128);
        if(v.exists) {
          sum += v.v;
          count++;
        }
      }

      measured.push_back(sum / count);
    }

    float corrCoeff = correlationCoefficient(
      obj.orthogonalValues.begin(), obj.orthogonalValues.end(),
      measured.begin(), measured.end(),
      CIRATEFI_ORTHOGONALS);
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
    Mat &img, int x, int y, float scale, float angle, float yCorrection) {
  ParallelMatchQuality result;

  float bestXCorrection = -1;
  float bestCorrelation = 0.0;

  for(float xCorrection = 0.8; xCorrection < 1.2; xCorrection += 0.01) {
    std::vector<float> measured;

    float wx = scale * xCorrection * sin(angle + 0.5 * M_PI);
    float wy = scale * xCorrection * cos(angle + 0.5 * M_PI);
    float hx = scale * yCorrection * sin(angle);
    float hy = scale * yCorrection * cos(angle);

    int sx = x - (obj.w / 2.0) * wx - (obj.h / 2.0) * hx;
    int sy = y - (obj.w / 2.0) * wy - (obj.h / 2.0) * hy;

    for(int n = 0; n < CIRATEFI_PARALLELS; ++n) {
      float sum = 0;
      float count = 0;

      float lx = obj.w * 1.0 * n / (CIRATEFI_PARALLELS - 1);
      for(int ly = 0; ly < obj.h; ++ly) {
        float ix = sx + lx * wx + ly * hx;
        float iy = sy + lx * wy + ly * hy;

        auto v = interpolate(img, ix, iy, 128);
        if(v.exists) {
          sum += v.v;
          count++;
        }
      }

      measured.push_back(sum / count);
    }

    float corrCoeff = correlationCoefficient(
      obj.parallelValues.begin(), obj.parallelValues.end(),
      measured.begin(), measured.end(),
      CIRATEFI_PARALLELS);
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
};

AllMatchQuality compareAll(const ObjectDescription &obj,
    Mat &img, int x, int y, float scale, float angle,
    float yCorrection, float xCorrection) {
  AllMatchQuality result;

  float bestCorrelation = 0.0;

  std::vector<float> measured;

  float wx = scale * xCorrection * sin(angle + 0.5 * M_PI);
  float wy = scale * xCorrection * cos(angle + 0.5 * M_PI);
  float hx = scale * yCorrection * sin(angle);
  float hy = scale * yCorrection * cos(angle);

  int sx = x - (obj.w / 2.0) * wx - (obj.h / 2.0) * hx;
  int sy = y - (obj.w / 2.0) * wy - (obj.h / 2.0) * hy;

  for(int ly = 0; ly < obj.h; ++ly) {
    for(int lx = 0; lx < obj.w; ++lx) {
      float sum = 0;
      int count = 0;

      for(float yy = ly; yy < ly + 1; yy += 0.3) {
        for(float xx = lx; xx < lx + 1; xx += 0.3) {
          float ix = sx + lx * wx + ly * hx;
          float iy = sy + lx * wy + ly * hy;

          auto v = interpolate(img, ix, iy, 255);
          if(!v.exists) {
            result.score = 0;
            return result;
          }

          sum += v.v;
          count++;
        }
      }

      measured.push_back(sum / count);
    }
  }

  float corrCoeff = correlationCoefficient(
    obj.allValues.begin(), obj.allValues.end(),
    measured.begin(), measured.end(),
    obj.h * obj.w);
  if(fabs(corrCoeff) > bestCorrelation) {
    bestCorrelation = fabs(corrCoeff);
  }

  result.score = bestCorrelation;
  return result;
}

int main(int, char**)
{
    Mat reference = imread("svo.0120.png", IMREAD_COLOR );
    // Mat query = imread("svo.0120.png", IMREAD_COLOR );
    // Mat query = imread("svo.0121.png", IMREAD_COLOR );
    Mat query = imread("svo.0011.png", IMREAD_COLOR );
    // Mat query = imread("svo.0123.png", IMREAD_COLOR );
    // Mat query = imread("svo.0115.png", IMREAD_COLOR );

    if(reference.empty() || query.empty()) {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }

    int sx = 355;
    int ex = 420;
    int sy = 135;
    int ey = 210;

    rectangle(reference,
              Point(sx, sy),
              Point(ex, ey),
              Scalar(0, 255, 0),
              1,
              8);

    auto hand = measureObject(reference, sx, sy, ex, ey);

    // for(int y = 250; y < 290; ++y) {
    for(int y = 0; y < query.rows; ++y) {
      cout << y << endl;
      // for(int x = 220; x < 260; ++x) {
      for(int x = 0; x < query.cols; ++x) {
    // do {
        // int x = (sx + ex) / 2;
        // int y = (sy + ey) / 2;
        auto circleMatch = compareCircles(hand, query, x, y);
        // cout << circleMatch.score << endl;
        if(circleMatch.score < CIRATEFI_CIRCLE_THRESHOLD) continue;

        query.at<rgb>(y, x).g = 40;

        auto radialMatch = compareRadials(hand, query, x, y, circleMatch.scale);
        if(radialMatch.score < CIRATEFI_RADIAL_THRESHOLD) continue;

        query.at<rgb>(y, x).g = 64;

        auto orthogonalMatch = compareOrthogonals(hand, query, x, y,
            circleMatch.scale, radialMatch.angle);
        if(orthogonalMatch.score < CIRATEFI_ORTHOGONAL_THRESHOLD) continue;

        query.at<rgb>(y, x).g = 80;

        auto parallelMatch = compareParallels(hand, query, x, y,
            circleMatch.scale, radialMatch.angle, orthogonalMatch.yCorrection);
        if(parallelMatch.score < CIRATEFI_PARALLEL_THRESHOLD) continue;

        query.at<rgb>(y, x).g = 128;

        auto allMatch = compareAll(hand, query, x, y,
            circleMatch.scale, radialMatch.angle,
            orthogonalMatch.yCorrection, parallelMatch.xCorrection);
        if(allMatch.score < CIRATEFI_ALL_THRESHOLD) continue;

        cout
          << " S: " << circleMatch.scale
          << " A: " << radialMatch.angle
          << " y: " << orthogonalMatch.yCorrection
          << " x: " << parallelMatch.xCorrection
          << " Q: " << radialMatch.score
          << " O: " << orthogonalMatch.score
          << " P: " << parallelMatch.score
          << " A: " << allMatch.score
          << endl;

        query.at<rgb>(y, x).g = 255;
      }
    };

    namedWindow("Display window", WINDOW_AUTOSIZE);
    imshow("Display window", reference);

    namedWindow("Display window 2", WINDOW_AUTOSIZE);
    imshow("Display window 2", query);

    waitKey(0);
    return 0;
}
