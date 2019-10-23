namespace gps {

struct Position {
    float latitude;
    float longitude;
};

bool has_fix();
Position get_current_position();

}
