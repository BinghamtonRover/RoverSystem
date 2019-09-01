#include <math.h>

#include "math.hpp"

namespace math {
	float cosf(float theta) {
		return ::cosf(theta);
	}

	float sinf(float theta) {
		return ::sinf(theta);
	}

	float powf(float value, float exp) {
		return ::powf(value, exp);
	}

	float sqrtf(float value) {
		return ::sqrtf(value);
	}
}
