#include <math.h>

#include "math.hpp"

namespace math {
	float cosf(float theta) {
		return ::cosf(theta);
	}

	float sinf(float theta) {
		return ::sinf(theta);
	}

    float tanf(float theta) {
        return ::tanf(theta);
    }
    
    float dtor(float deg) {
        return deg * PI / 180.0f;
    }

	float powf(float value, float exp) {
		return ::powf(value, exp);
	}

	float sqrtf(float value) {
		return ::sqrtf(value);
	}
}
