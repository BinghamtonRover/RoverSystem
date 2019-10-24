#include "lidar.hpp"

namespace lidar {

Error start(const char* ip) {
    return Error::OK;
}

Error scan(std::vector<long>& data_points) {
    // Just put 271 zeros in there.
    for (int i = 0; i < 271; i++) {
        data_points.push_back(0);
    }

    return Error::OK;
}

} // namespace lidar
