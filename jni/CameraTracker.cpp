#include <jni.h>
#include "CameraTracker.h"

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>
#include <kalman/ukf.hpp>
#include <Eigen/Geometry>

#include <android/log.h>

// #define DUMP_IMAGE 1
#if DUMP_IMAGE
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <fcntl.h>
#endif

#define MAP_SCALE 2.0

using namespace Eigen;

// state:
static constexpr int X_x = 0;
static constexpr int X_y = 1;
static constexpr int X_z = 2;
static constexpr int V_x = 3;
static constexpr int V_y = 4;
static constexpr int V_z = 5;
static constexpr int A_x = 6;
static constexpr int A_y = 7;
static constexpr int A_z = 8;
static constexpr int rotv_x = 9;
static constexpr int rotv_y = 10;
static constexpr int rotv_z = 11;
static constexpr int g_x = 12;
static constexpr int g_y = 13;
static constexpr int g_z = 14;
static constexpr int MapScale = 15;

static constexpr int D = 16;

typedef Matrix<double, D, 1> V;
typedef Matrix<double, D, D> M;

static vk::AbstractCamera *camera;
static int cameraWidth;
static int cameraHeight;
static svo::FrameHandlerMono *frameHandler;
static double frameTime; // TODO: Forward from real camera
static unsigned char *intensitiesBuffer;
static std::mutex sensorFusionLock;

struct SensorFusion: public UnscentedKalmanFilter<D> {
  public:
    SensorFusion(const V &v, const M &m): UnscentedKalmanFilter(v, m) {}

    double rot_x, rot_y, rot_z, rot_w;

    // Euler for quaternion rotation, because I'm confused
    void predictRot(double dt) {
      double vx = state().x(rotv_x) / 180 * 2 * M_PI;
      double vy = -state().x(rotv_y) / 180 * 2 * M_PI;
      double vz = -state().x(rotv_z) / 180 * 2 * M_PI;

      double omegaMagnitude = sqrt(vx * vx + vy * vy + vz * vz);

      // Normalize the rotation vector if it's big enough to get the axis
      if (omegaMagnitude > 1e-9) {
          vx /= omegaMagnitude;
          vy /= omegaMagnitude;
          vz /= omegaMagnitude;
      }

      // Integrate around this axis with the angular speed by the time step
      // in order to get a delta rotation from this sample over the time step
      // We will convert this axis-angle representation of the delta rotation
      // into a quaternion before turning it into the rotation matrix.
      double thetaOverTwo = omegaMagnitude * dt / 2.0f;
      double sinThetaOverTwo = sin(thetaOverTwo);
      double cosThetaOverTwo = cos(thetaOverTwo);
      Quaternion<double> qdt(
        cosThetaOverTwo,
        sinThetaOverTwo * vx,
        sinThetaOverTwo * vy,
        sinThetaOverTwo * vz
      );

      // const double dx = x(rotv_x) * dt / 2;
      // const double dy = x(rotv_y) * dt / 2;
      // const double dz = x(rotv_z) * dt / 2;

      // const double rv_w = cos(dx) * cos(dy) * cos(dz) + sin(dx) * sin(dy) * sin(dz);
      // const double rv_x = sin(dx) * cos(dy) * cos(dz) - cos(dx) * sin(dy) * sin(dz);
      // const double rv_y = cos(dx) * sin(dy) * cos(dz) + sin(dx) * cos(dy) * sin(dz);
      // const double rv_z = cos(dx) * cos(dy) * sin(dz) - sin(dx) * sin(dy) * cos(dz);
      // Quaternion<double> qdt(rv_w, rv_x, rv_y, rv_z);

      Quaternion<double> q(rot_w, rot_x, rot_y, rot_z);

      __android_log_print(ANDROID_LOG_INFO, "Tracker", "q XYZW: %lf %lf %lf %lf",
          q.x(), q.y(), q.z(), q.w());
      __android_log_print(ANDROID_LOG_INFO, "Tracker", "qdt XYZW: %lf %lf %lf %lf",
          qdt.x(), qdt.y(), qdt.z(), qdt.w());

      q = q * qdt;

      const double new_mag = sqrt(
          q.w() * q.w() +
          q.x() * q.x() +
          q.y() * q.y() +
          q.z() * q.z()
      );

      rot_w = q.w() / new_mag;
      rot_x = q.x() / new_mag;
      rot_y = q.y() / new_mag;
      rot_z = q.z() / new_mag;

      __android_log_print(ANDROID_LOG_INFO, "Tracker", "final rotXYZW: %lf %lf %lf %lf",
          rot_x,
          rot_y,
          rot_z,
          rot_w);
    }
};
SensorFusion *sensorFusion;

static long long int sensorFusionTimestamp = -1;
static V predictionNoise;
static M predictionNoiseCovariance;
static Matrix<double, 3, 1> frameMeasureNoise;
static Matrix<double, 3, 3> frameMeasureNoiseCovariance;
static Matrix<double, 3, 1> accelerometerMeasureNoise;
static Matrix<double, 3, 3> accelerometerMeasureNoiseCovariance;
static Matrix<double, 3, 1> gyroscopeMeasureNoise;
static Matrix<double, 3, 3> gyroscopeMeasureNoiseCovariance;
static bool trackingEstablished;

JNIEXPORT void JNICALL Java_name_drahflow_ar_CameraTracker_SVO_1prepare
  (JNIEnv *, jclass, jint width, jint height, jfloat, jfloat, jdouble, jdouble) {
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "Tracker starting...");

  cameraWidth = width;
  cameraHeight = height;

  camera = new vk::PinholeCamera(width, height,
      540.5454, 539.3325, 318.0489, 237.989,
      0.1272545, -0.4216122, -0.0006996348, -0.014160222, 0.61963481);
  frameHandler = new svo::FrameHandlerMono(camera);
  frameHandler->start();
  frameTime = 0;
  trackingEstablished = false;

  intensitiesBuffer = new unsigned char[cameraWidth * cameraHeight];

  predictionNoise = V::Zero();
  predictionNoiseCovariance <<
    1e-6,    0,    0, 1e-7,    0,    0,    0,    0,    0,    0,   0,   0,   0,    0,    0,    0,
       0, 1e-6,    0,    0, 1e-7,    0,    0,    0,    0,    0,   0,   0,   0,    0,    0,    0,
       0,    0, 1e-6,    0,    0, 1e-7,    0,    0,    0,    0,   0,   0,   0,    0,    0,    0,
    1e-7,    0,    0, 1e-9,    0,    0, 1e-6,    0,    0,    0,   0,   0,   0,    0,    0,    0,
       0, 1e-7,    0,    0, 1e-9,    0,    0, 1e-6,    0,    0,   0,   0,   0,    0,    0,    0,
       0,    0, 1e-7,    0,    0, 1e-9,    0,    0, 1e-6,    0,   0,   0,   0,    0,    0,    0,
       0,    0,    0, 1e-6,    0,    0,  0.1,    0,    0,    0,   0,   0,   0,    0,    0,    0,
       0,    0,    0,    0, 1e-6,    0,    0,  0.1,    0,    0,   0,   0,   0,    0,    0,    0,
       0,    0,    0,    0,    0, 1e-6,    0,    0,  0.1,    0,   0,   0,   0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,  1e2,   0,   0,   0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 1e2,   0,   0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   0, 1e2,   0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   0,   0,1e-0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   0,   0,   0, 1e-0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   0,   0,   0,    0, 1e-0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   0,   0,   0,    0,    0, 1e-9;

  frameMeasureNoise = Matrix<double, 3, 1>::Zero();
  frameMeasureNoiseCovariance <<
    1e-5,    0,    0,
       0, 1e-5,    0,
       0,    0, 1e-5;

  accelerometerMeasureNoise = Matrix<double, 3, 1>::Zero();
  accelerometerMeasureNoiseCovariance <<
    1e-3, 0, 0,
    0, 1e-3, 0,
    0, 0, 1e-3;

  gyroscopeMeasureNoise = Matrix<double, 3, 1>::Zero();
  gyroscopeMeasureNoiseCovariance <<
    1e-9, 0, 0,
    0, 1e-9, 0,
    0, 0, 1e-9;
}

static V sensorModelODE(const V &x) {
  V xdt;
  xdt(X_x) = x(V_x);
  xdt(X_y) = x(V_y);
  xdt(X_z) = x(V_z);
  xdt(V_x) = x(A_x) - 0.1 * x(V_x);
  xdt(V_y) = x(A_y) - 0.1 * x(V_y);
  xdt(V_z) = x(A_z) - 0.1 * x(V_z);
  xdt(A_x) = 0;
  xdt(A_y) = 0;
  xdt(A_z) = 0;
  xdt(rotv_x) = 0;
  xdt(rotv_y) = 0;
  xdt(rotv_z) = 0;
  xdt(g_x) = 0;
  xdt(g_y) = 0;
  xdt(g_z) = 0;
  xdt(MapScale) = 0;
  return xdt;
}

static std::function<V(const V&)> sensorModelRungeKutta(double dt) {
  const auto f = std::function<V(const V&)>(sensorModelODE);

  return [f,dt](const V &x) {
    auto k1 = f(x);
    auto k2 = f(x + dt/2 * k1);
    auto k3 = f(x + dt/2 * k2);
    auto k4 = f(x + dt * k3);

    V ret = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);

    return ret;
  };
}

static void initializeKalmanFilter() {
  V initialState;
  initialState <<
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 9.81, 0,
    4;

  M initialCovariance;
  initialCovariance <<
    30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0,
    0, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0,
    0, 0, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0,
    0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0,
    0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0,
    0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,  1, 0, 0,  0,  0,  0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 1, 0,  0,  0,  0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 1,  0,  0,  0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0,1e5,  0,  0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0,  0,1e5,  0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0,  0,  0,1e5, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0,  0,  0,  0, 1e-5;

  sensorFusion = new SensorFusion(initialState, initialCovariance);

  sensorFusion->rot_w = 1;
  sensorFusion->rot_x = 0;
  sensorFusion->rot_y = 0;
  sensorFusion->rot_z = 0;
}

static void updateKalmanFilter(long long time_nano) {
  std::lock_guard<std::mutex> lock(sensorFusionLock);

  if(sensorFusionTimestamp < 0) {
    initializeKalmanFilter();
  } else {
    while(sensorFusionTimestamp < time_nano) {
      // maybe also replay gyro history?

      __android_log_print(ANDROID_LOG_INFO, "Tracker", "t_sensor: %lld   t_frame: %lld",
          sensorFusionTimestamp, time_nano);

      const long int dt_int = std::min(time_nano - sensorFusionTimestamp, 10000000ll);
      const double dt = dt_int / 100000000.0;

      sensorFusion->predict(predictionNoise, predictionNoiseCovariance, sensorModelRungeKutta(dt));
      sensorFusion->predictRot(dt);

      sensorFusionTimestamp += dt_int;
    }
  }

  sensorFusionTimestamp = time_nano;
}

static void saveTransformation(JNIEnv *env, SensorFusion &filter, jfloatArray transformation) {
  jsize transformationLen = env->GetArrayLength(transformation);
  if(transformationLen != 7) {
    throw std::runtime_error("Transformation output buffer is not 7 elements long.");
  }

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEA");

  jfloat *transformationData = env->GetFloatArrayElements(transformation, 0);
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMED");
  transformationData[0] = filter.state().x(X_x) / MAP_SCALE; //filter.state().x(MapScale);
  transformationData[1] = filter.state().x(X_y) / MAP_SCALE; //filter.state().x(MapScale);
  transformationData[2] = filter.state().x(X_z) / MAP_SCALE; //filter.state().x(MapScale);
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEE");
  transformationData[3] = filter.rot_x;
  transformationData[4] = filter.rot_y;
  transformationData[5] = filter.rot_z;
  transformationData[6] = filter.rot_w;
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEF");
  env->ReleaseFloatArrayElements(transformation, transformationData, 0);
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_CameraTracker_SVO_1processFrame
  (JNIEnv *env, jclass, jfloatArray intensities, jfloatArray transformation,
   jlong time_nano) {
  // trackingEstablished = true;
  // updateKalmanFilter(time_nano);
  // return; // This + above lines are FIXME

  jsize intensitiesLen = env->GetArrayLength(intensities);
  if(intensitiesLen != cameraWidth * cameraHeight) {
    throw std::runtime_error("Frame has invalid size.");
  }

  jfloat *intensitiesData = env->GetFloatArrayElements(intensities, 0);
  for(int i = 0; i < cameraWidth * cameraHeight; ++i) {
    intensitiesBuffer[i] = intensitiesData[i] * 255;
  }

  #if DUMP_IMAGE
  char nameBuf[128];
  sprintf(nameBuf, "/sdcard/svo.%04d.raw", (int)(frameTime * 200));
  int fd = open(nameBuf, O_CREAT | O_TRUNC | O_WRONLY);
  if(fd > 0) {
    write(fd, intensitiesBuffer, cameraWidth * cameraHeight);
    close(fd);
  }
  #endif

  cv::Mat img(cameraHeight, cameraWidth, CV_8UC1, intensitiesBuffer);
  frameHandler->addImage(img, frameTime += 0.01);

  env->ReleaseFloatArrayElements(intensities, intensitiesData, 0);

  if(frameHandler->trackingQuality() != svo::FrameHandlerMono::TrackingQuality::TRACKING_GOOD) {
    return;
  }

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEB");
  // FIXME: This can apparently die...
  auto pose = frameHandler->lastFrame()->T_f_w_.inverse();
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEC");
  auto trans = pose.translation();
  auto rot = pose.unit_quaternion();

  trackingEstablished = true;
  updateKalmanFilter(time_nano);

  Matrix<double, 3, 1> observation;
  observation <<
    trans[0],
    trans[1],
    trans[2];

  {
    std::lock_guard<std::mutex> lock(sensorFusionLock);

    sensorFusion->update<3>(frameMeasureNoise, frameMeasureNoiseCovariance,
        [](const V &x) -> Matrix<double, 3, 1> {
        Matrix<double, 3, 1> ret;
          ret <<
            x(X_x),
            x(X_y),
            x(X_z);
          return ret;
        }, observation);

    sensorFusion->rot_x = rot.x();
    sensorFusion->rot_y = rot.y();
    sensorFusion->rot_z = rot.z();
    sensorFusion->rot_w = rot.w();

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "XYZ: %lf %lf %lf",
      sensorFusion->state().x(X_x),
      sensorFusion->state().x(X_y),
      sensorFusion->state().x(X_z));
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "vXYZ: %lf %lf %lf",
      sensorFusion->state().x(V_x),
      sensorFusion->state().x(V_y),
      sensorFusion->state().x(V_z));
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "aXYZ: %lf %lf %lf",
      sensorFusion->state().x(A_x),
      sensorFusion->state().x(A_y),
      sensorFusion->state().x(A_z));
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "sigma(XYZ): %lf %lf %lf",
      sensorFusion->state().P(X_x, X_x),
      sensorFusion->state().P(X_y, X_y),
      sensorFusion->state().P(X_z, X_z));
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "rot: %lf %lf %lf %lf",
      sensorFusion->rot_x,
      sensorFusion->rot_y,
      sensorFusion->rot_z,
      sensorFusion->rot_w);

    saveTransformation(env, *sensorFusion, transformation);
  }

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "ID: %d, #Features: %d, took %lf ms",
      frameHandler->lastFrame()->id_,
      frameHandler->lastNumObservations(),
      frameHandler->lastProcessingTime() * 1000);
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_CameraTracker_SVO_1processAccelerometer
  (JNIEnv *env, jclass, jfloatArray xyz, jlong time_nano) {
   if(!trackingEstablished) return;

   jsize xyzLen = env->GetArrayLength(xyz);
   if(xyzLen != 3) {
     throw std::runtime_error("Accelerometer data has invalid size.");
   }

   updateKalmanFilter(time_nano);

   // __android_log_print(ANDROID_LOG_INFO, "Tracker", "aXYZ: %lf %lf %lf",
   //     sensorFusion->state().x(A_x),
   //     sensorFusion->state().x(A_y),
   //     sensorFusion->state().x(A_z));
   // __android_log_print(ANDROID_LOG_INFO, "Tracker", "sigma(aXYZ): %lf %lf %lf",
   //     sensorFusion->state().P(A_x, A_x),
   //     sensorFusion->state().P(A_y, A_y),
   //     sensorFusion->state().P(A_z, A_z));
   // __android_log_print(ANDROID_LOG_INFO, "Tracker", "gXYZ: %lf %lf %lf",
   //     sensorFusion->state().x(g_x),
   //     sensorFusion->state().x(g_y),
   //     sensorFusion->state().x(g_z));
   // __android_log_print(ANDROID_LOG_INFO, "Tracker", "Map Scale: %lf", sensorFusion->state().x(MapScale));
   // __android_log_print(ANDROID_LOG_INFO, "Tracker", "sigma(Map Scale): %lf", sensorFusion->state().P(MapScale, MapScale));
   __android_log_print(ANDROID_LOG_INFO, "Tracker", "Updating from accelerometer...");


   jfloat *xyzData = env->GetFloatArrayElements(xyz, 0);

   __android_log_print(ANDROID_LOG_INFO, "Tracker", "accelXYZ: %f %f %f",
       xyzData[0], xyzData[1], xyzData[2]);

   Matrix<double, 3, 1> observation;
   observation <<
     xyzData[0],
     xyzData[1],
     xyzData[2];

   env->ReleaseFloatArrayElements(xyz, xyzData, 0);

   sensorFusion->update<3>(accelerometerMeasureNoise, accelerometerMeasureNoiseCovariance,
       [](const V &x) -> Matrix<double, 3, 1> {
         Matrix<double, 3, 1> ret;

         double rel_ax = x(A_x) / MAP_SCALE + x(g_x);
         double rel_ay = -x(A_y) / MAP_SCALE + x(g_y);
         double rel_az = -x(A_z) / MAP_SCALE + x(g_z);

         // signs randomly inverted until gravity vector became semi-stable
         Quaternion<double> q(sensorFusion->rot_w, -sensorFusion->rot_x, sensorFusion->rot_y, sensorFusion->rot_z);
         Quaternion<double> accel(0, rel_ax, rel_ay, rel_az);

         Quaternion<double> rotated = q * accel * q.inverse();

         ret <<
           rotated.x() / 9.81 * 9.69,
           rotated.y() / 9.81 * 9.676,
           rotated.z() / 9.81 * 9.00;
         return ret;
       }, observation);

   __android_log_print(ANDROID_LOG_INFO, "Tracker", "gXYZ: %lf %lf %lf",
       sensorFusion->state().x(g_x),
       sensorFusion->state().x(g_y),
       sensorFusion->state().x(g_z));
   __android_log_print(ANDROID_LOG_INFO, "Tracker", "aXYZ: %lf %lf %lf",
       sensorFusion->state().x(A_x),
       sensorFusion->state().x(A_y),
       sensorFusion->state().x(A_z));
   __android_log_print(ANDROID_LOG_INFO, "Tracker", "sigma(aXYZ): %lf %lf %lf",
       sensorFusion->state().P(A_x, A_x),
       sensorFusion->state().P(A_y, A_y),
       sensorFusion->state().P(A_z, A_z));
   __android_log_print(ANDROID_LOG_INFO, "Tracker", "Map Scale: %lf", sensorFusion->state().x(MapScale));
   __android_log_print(ANDROID_LOG_INFO, "Tracker", "sigma(Map Scale): %lf", sensorFusion->state().P(MapScale, MapScale));
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_CameraTracker_SVO_1processGyroscope
  (JNIEnv *env, jclass, jfloatArray xyz, jlong time_nano) {
  if(!trackingEstablished) return;

  jsize xyzLen = env->GetArrayLength(xyz);
  if(xyzLen != 3) {
    throw std::runtime_error("Gyroscope data has invalid size.");
  }

  jfloat *xyzData = env->GetFloatArrayElements(xyz, 0);

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "gyr data XYZ: %f %f %f",
      xyzData[0], xyzData[1], xyzData[2]);

  Matrix<double, 3, 1> observation;
  observation << xyzData[0], xyzData[1], xyzData[2];

  env->ReleaseFloatArrayElements(xyz, xyzData, 0);

  long long int middle_time = 0;
  {
    std::lock_guard<std::mutex> lock(sensorFusionLock);
    middle_time = (time_nano + sensorFusionTimestamp) / 2;
  }

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "sensorFusionTimestamp: %lld", sensorFusionTimestamp);
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "middle_time: %lld", middle_time);
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "time_nano: %lld", time_nano);

  updateKalmanFilter(middle_time);

  {
    std::lock_guard<std::mutex> lock(sensorFusionLock);

    sensorFusion->update<3>(gyroscopeMeasureNoise, gyroscopeMeasureNoiseCovariance,
        [](const V &x) -> Matrix<double, 3, 1> {
          Matrix<double, 3, 1> ret;
          ret <<
            x(rotv_x),
            x(rotv_y),
            x(rotv_z);
          return ret;
        }, observation);
  }

  updateKalmanFilter(time_nano);

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "time_nano: %lld", time_nano);
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_CameraTracker_SVO_1getTransformation
  (JNIEnv *env, jclass, jlong time_nano, jfloatArray transformation) {
  if(!trackingEstablished) return;

  long long int tmpTimestamp;

  SensorFusion tmp(V::Zero(), M::Zero());
  {
    std::lock_guard<std::mutex> lock(sensorFusionLock);
    tmp = *sensorFusion;
    tmpTimestamp = sensorFusionTimestamp;
  }

  while(tmpTimestamp < time_nano) {
    // maybe also replay gyro history?

    const long int dt_int = std::min(time_nano - tmpTimestamp, 10000000ll);
    const double dt = dt_int / 100000000.0;
    tmp.predict(predictionNoise, predictionNoiseCovariance, sensorModelRungeKutta(dt));
    tmp.predictRot(dt);

    tmpTimestamp += dt_int;
  }

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "est. XYZ: %lf %lf %lf",
      tmp.state().x(X_x),
      tmp.state().x(X_y),
      tmp.state().x(X_z));

  saveTransformation(env, tmp, transformation);
}
