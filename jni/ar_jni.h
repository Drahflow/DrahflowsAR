#include <jni.h>
/* Header for class name_drahflow_ar_JNI */

#ifndef _Included_name_drahflow_ar_JNI
#define _Included_name_drahflow_ar_JNI
#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_SVO_1prepare
  (JNIEnv *, jclass, jint, jint, jfloat, jfloat, jdouble, jdouble);

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_SVO_1processFrame
  (JNIEnv *, jclass, jfloatArray, jlong);

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_SVO_1processAccelerometer
  (JNIEnv *, jclass, jfloatArray, jlong);

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_SVO_1processGyroscope
  (JNIEnv *, jclass, jfloatArray, jlong);

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_SVO_1getTransformation
  (JNIEnv *, jclass, jlong, jfloatArray);

JNIEXPORT jboolean JNICALL Java_name_drahflow_ar_JNI_SVO_1hasGoodTracking
  (JNIEnv *, jclass);

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_SVO_1setAccelerometerScale
  (JNIEnv *, jclass, jfloat);


JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1setMarker
  (JNIEnv *, jclass, jint, jint, jint, jint);

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1processFrame
  (JNIEnv *, jclass, jfloatArray);

JNIEXPORT void JNICALL Java_name_drahflow_ar_JNI_Gesture_1getTransformationRelative
  (JNIEnv *, jclass, jlong, jfloatArray);

#ifdef __cplusplus
}
#endif
#endif
