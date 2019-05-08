#include "science_onboard.hpp"

#include <cstdio>

namespace science{

Error init(const char* device_serial_id){
	return Error::OK;
}

Error update(float Altitude, float Moisture, float Humidity, float Temperature, float Heat_Index){
	return Error::OK;
}

}
