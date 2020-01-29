#ifndef MAP_H
#define MAP_H

struct MapObstacle {
    float* vertices;
    size_t num_vertices;
};

struct Map {
    MapObstacle* obstacles;
    size_t num_obstacles;

    float target_x, target_y;
};

#define MAP_ERROR_DEF(X) \
    X(MAP_ERROR_OK), \
    X(MAP_ERROR_OPEN_FILE), \
    X(MAP_ERROR_PARSE), \
    X(MAP_ERROR_FORMAT)

#define X(name) name
enum MapError {
    MAP_ERROR_DEF(X)
};
#undef X

const char* map_error_string(MapError e);

MapError map_load_from_file(Map* map, const char* file_path);

#endif
