#include <vector>

namespace lidar {

constexpr int BUFFER_SIZE = 5000;

enum class Error {
	OK,

	BIND,
	CONNECT,

};

Error start(const char* ip);

Error scan(std::vector<long>& data_points);

}
