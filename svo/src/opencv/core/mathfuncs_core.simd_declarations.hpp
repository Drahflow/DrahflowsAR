#include "precomp.hpp"

#define CV_CPU_SIMD_FILENAME "../../../../src/opencv/core/mathfuncs_core.simd.hpp"

#define CV_CPU_DISPATCH_MODE BASELINE
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"
#define CV_CPU_DISPATCH_MODE SSSE3
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"

#define CV_CPU_DISPATCH_MODES_ALL BASELINE, SSSE3
