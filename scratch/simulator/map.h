#ifndef MAP_H
#define MAP_H

typedef struct {
    float* vertices;
    size_t num_vertices;
} MapObstacle;

typedef struct {
    MapObstacle* obstacles;
    size_t num_obstacles;

    float target_x, target_y;
} Map;

#define MAP_ERROR_DEF(X) \
    X(MAP_ERROR_OK), \
    X(MAP_ERROR_OPEN_FILE), \
    X(MAP_ERROR_PARSE), \
    X(MAP_ERROR_FORMAT)

#define X(name) name
typedef enum {
    MAP_ERROR_DEF(X)
} MapError;
#undef X

const char* map_error_string(MapError e);

MapError map_load_from_file(Map* map, const char* file_path);

#endif
