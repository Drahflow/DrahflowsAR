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

  camera = new vk::PinholeCamera(width, height, 315.5, 315.5, 376.0, 240.0);
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
  __android_log_print(ANDROID_LOG_INFO, "Tracker", "INTENSITY: %d",
      static_cast<int>(intensitiesBuffer[1024]));

  cv::Mat img(cameraHeight, cameraWidth, CV_8UC1, intensitiesBuffer);
  frameHandler->addImage(img, frameTime += 0.01);

  __android_log_print(ANDROID_LOG_INFO, "Tracker", "ID: %d, #Features: %d, took %lf μs",
      frameHandler->lastFrame()->id_,
      frameHandler->lastNumObservations(),
      frameHandler->lastProcessingTime());
}
