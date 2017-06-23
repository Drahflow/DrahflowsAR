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

#include <android/log.h>

// #define DUMP_IMAGE 1
#if DUMP_IMAGE
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <fcntl.h>
#endif

static vk::AbstractCamera *camera;
static int cameraWidth;
static int cameraHeight;
static svo::FrameHandlerMono *frameHandler;
static double frameTime; // TODO: Forward from real camera
static unsigned char *intensitiesBuffer;

JNIEXPORT void JNICALL Java_name_drahflow_ar_CameraTracker_SVO_1prepare
  (JNIEnv *, jclass, jint width, jint height, jfloat, jfloat, jdouble, jdouble) {
  cameraWidth = width;
  cameraHeight = height;

  camera = new vk::PinholeCamera(width, height,
      540.5454, 539.3325, 318.0489, 237.989,
      0.1272545, -0.4216122, -0.0006996348, -0.014160222, 0.61963481);
  frameHandler = new svo::FrameHandlerMono(camera);
  frameHandler->start();
  frameTime = 0;

  intensitiesBuffer = new unsigned char[cameraWidth * cameraHeight];
}

JNIEXPORT void JNICALL Java_name_drahflow_ar_CameraTracker_SVO_1processFrame
  (JNIEnv *env, jclass, jfloatArray intensities, jfloatArray transformation) {
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

  jsize transformationLen = env->GetArrayLength(transformation);
  if(transformationLen != 7) {
    throw std::runtime_error("Transformation output buffer is not 7 elements long.");
  }

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEA");

  jfloat *transformationData = env->GetFloatArrayElements(transformation, 0);
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEB");
  auto pose = frameHandler->lastFrame()->T_f_w_.inverse();
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEC");
  auto trans = pose.translation();
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMED");
  transformationData[0] = trans[0];
  transformationData[1] = trans[1];
  transformationData[2] = trans[2];
  auto rot = pose.unit_quaternion();
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEE");
  transformationData[3] = rot.x();
  transformationData[4] = rot.y();
  transformationData[5] = rot.z();
  transformationData[6] = rot.w();
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "FIXMEF");
  env->ReleaseFloatArrayElements(transformation, transformationData, 0);

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "ID: %d, #Features: %d, took %lf ms",
      frameHandler->lastFrame()->id_,
      frameHandler->lastNumObservations(),
      frameHandler->lastProcessingTime() * 1000);
}
