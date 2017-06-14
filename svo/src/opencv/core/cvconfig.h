#ifndef OPENCV_CVCONFIG_H
#define OPENCV_CVCONFIG_H

#define HAVE_EIGEN 1
#define CV_CPU_HAS_SUPPORT_SSE 1
#define CV_CPU_HAS_SUPPORT_SSE2 1
#define CV_CPU_HAS_SUPPORT_SSE3 1
#define CV_CPU_HAS_SUPPORT_SSSE3 1
#define CV_CPU_BASELINE_FEATURES CPU_MMX, CPU_SSE, CPU_SSE2, CPU_SSE3, CPU_SSSE3
//         g_hwFeatureNames[CPU_MMX] = "MMX";
//         g_hwFeatureNames[CPU_SSE] = "SSE";
//         g_hwFeatureNames[CPU_SSE2] = "SSE2";
//         g_hwFeatureNames[CPU_SSE3] = "SSE3";
//         g_hwFeatureNames[CPU_SSSE3] = "SSSE3";
//         g_hwFeatureNames[CPU_SSE4_1] = "SSE4.1";
//         g_hwFeatureNames[CPU_SSE4_2] = "SSE4.2";
//         g_hwFeatureNames[CPU_POPCNT] = "POPCNT";
//         g_hwFeatureNames[CPU_FP16] = "FP16";
//         g_hwFeatureNames[CPU_AVX] = "AVX";
//         g_hwFeatureNames[CPU_AVX2] = "AVX2";
//         g_hwFeatureNames[CPU_FMA3] = "FMA3";
// 
//         g_hwFeatureNames[CPU_AVX_512F] = "AVX512F";
//         g_hwFeatureNames[CPU_AVX_512BW] = "AVX512BW";
//         g_hwFeatureNames[CPU_AVX_512CD] = "AVX512CD";
//         g_hwFeatureNames[CPU_AVX_512DQ] = "AVX512DQ";
//         g_hwFeatureNames[CPU_AVX_512ER] = "AVX512ER";
//         g_hwFeatureNames[CPU_AVX_512IFMA512] = "AVX512IFMA";
//         g_hwFeatureNames[CPU_AVX_512PF] = "AVX512PF";
//         g_hwFeatureNames[CPU_AVX_512VBMI] = "AVX512VBMI";
//         g_hwFeatureNames[CPU_AVX_512VL] = "AVX512VL";
// 
//         g_hwFeatureNames[CPU_NEON] = "NEON";

#define CV_OCL_RUN(condition, func, ...)
#define CV_OVX_RUN(condition, func, ...)
#define CV_CPU_CALL_FP16(condition, func, ...)

#define HAVE_JPEG
#define HAVE_PNG

#endif
