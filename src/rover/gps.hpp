namespace gps {

enum class Error {
    OK,
    OPEN
};

Error open();

struct Position {
    float latitude;
    float longitude;
};

bool has_fix();
Position get_current_position();

} // namespace gps
