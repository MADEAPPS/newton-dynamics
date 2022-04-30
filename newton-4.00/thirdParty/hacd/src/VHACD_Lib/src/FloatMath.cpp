#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include "FloatMath.h"
#include <vector>

namespace nd
{
	namespace FLOAT_MATH
	{
		#define REAL float
		#undef REAL
		#define REAL double
	}
}
#include "FloatMath.inl"
