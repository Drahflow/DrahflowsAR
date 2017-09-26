#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

struct __attribute__((__packed__)) rgb {
  unsigned char b, g, r;
};

template<typename I> class circulator {
  public:
    circulator(const I &begin, const I &end): begin(begin), end(end), i(begin) { }

    typename I::value_type operator * () { return *i; }
    circulator &operator ++ () { ++i; loop(); return *this; }
    circulator operator ++ (int) {
      circulator ret = *this;
      i++;
      return ret;
    }

    circulator operator + (int d) {
      circulator ret = *this;
      ret.i += d;
      return ret;
    }

    bool operator == (const circulator &other) {
      return *this == &other || i == other.i;
    }

    bool operator != (const circulator &other) {
      return *this != &other && i != other.i;
    }

    const circulator &forever() {
      return infinity;
    }

  private:
    I begin, end, i;

    void loop() {
      if(i == end) i = begin;
    }

    static circulator infinity;
};

struct ObjectDescription {
  float w, h;
  std::vector<float> circleValues;
  std::vector<float> radialValues;
};

const int CIRATEFI_CIRCLES = 15;
const int CIRATEFI_RADIALS = 32;
const float CIRATEFI_CIRCLE_DIVISOR = 1.05;
const float CIRATEFI_CIRCLE_THRESHOLD = 0.95;
const float CIRATEFI_RADIAL_THRESHOLD = 0.95;
const float CIRATEFI_MAGNIFICATION_FACTOR = 3;
const float CIRATEFI_MINIFICATION_FACTOR = 0.3;
const float CIRATEFI_RADIAL_DENSITY = 0.5;
const float CIRATEFI_CONTRAST_THRESHOLD = 100;

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
      auto v = interpolate(img, cx + ri * sin(alpha), cy + ri * cos(alpha), 128);
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
      auto v = interpolate(img, cx + ri * sin(alpha), cy + ri * cos(alpha), 255);
      if(v.exists) {
        sum += v.v;
        count++;
      }
    }
    alpha += 2 * M_PI / CIRATEFI_RADIALS;

    result.radialValues.push_back(sum / count);
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
    const I &is, const I &ie, const J &js, const J &je) {
  const typename I::value_type xm = mean(is, ie);
  const typename J::value_type ym = mean(js, je);

  const typename I::value_type xx = meanDotProduct(is, ie, xm, is, ie, xm);
  const typename J::value_type yy = meanDotProduct(js, je, ym, js, je, ym);
  const typename I::value_type xy = meanDotProduct(is, ie, xm, js, je, ym);

  if(yy < CIRATEFI_CONTRAST_THRESHOLD || xx < CIRATEFI_CONTRAST_THRESHOLD) {
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
      measured.begin() + i, measured.begin() + i + CIRATEFI_CIRCLES);
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
      auto v = interpolate(img, cx + ri * sin(alpha), cy + ri * cos(alpha), 255);
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

  circulator<decltype(measured.begin())> rotating(measured.begin(), measured.end());

  for(size_t i = 0; i < measured.size(); ++i) {
    float corrCoeff = correlationCoefficient(
      obj.radialValues.begin(), obj.radialValues.end(),
      rotating + i, rotating.forever());
    if(fabs(corrCoeff) > bestCorrelation) {
      bestCorrelation = fabs(corrCoeff);
      bestIndex = i;
    }
  }

  // result.score = bestCorrelation;
  // result.scale = CIRATEFI_MAGNIFICATION_FACTOR / pow(CIRATEFI_CIRCLE_DIVISOR, bestIndex);
  return result;
}

int main(int, char**)
{
    Mat reference = imread("svo.0120.png", IMREAD_COLOR );
    Mat query = imread("svo.0120.png", IMREAD_COLOR );
    // Mat query = imread("svo.0121.png", IMREAD_COLOR );
    // Mat query = imread("svo.0011.png", IMREAD_COLOR );

    if(reference.empty() || query.empty()) {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }

    int sx = 355;
    int ex = 430;
    int sy = 135;
    int ey = 210;

    rectangle(reference,
              Point(sx, sy),
              Point(ex, ey),
              Scalar(0, 255, 0),
              1,
              8);

    auto hand = measureObject(reference, sx, sy, ex, ey);

    // for(int y = 0; y < query.rows; ++y) {
    //   cout << y << endl;
    //   for(int x = 0; x < query.cols; ++x) {
    do {
        int x = (sx + ex) / 2;
        int y = (sy + ey) / 2;
        auto circleMatch = compareCircles(hand, query, x, y);
        // cout << circleMatch.score << endl;
        if(circleMatch.score < CIRATEFI_CIRCLE_THRESHOLD) continue;

        query.at<rgb>(y, x).g = 128;

        auto radialMatch = compareRadials(hand, query, x, y, circleMatch.scale);
        cout << radialMatch.score << endl;
        if(radialMatch.score < CIRATEFI_RADIAL_THRESHOLD) continue;

        query.at<rgb>(y, x).g = 255;
      //}
    } while(0);

    namedWindow("Display window", WINDOW_AUTOSIZE);
    imshow("Display window", reference);

    namedWindow("Display window 2", WINDOW_AUTOSIZE);
    imshow("Display window 2", query);

    waitKey(0);
    return 0;
}
