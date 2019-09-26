#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tomlc99/toml.h"

#include "map.h"

#define X(name) [name] = #name
const char* map_error_names[] = {
   MAP_ERROR_DEF(X) 
};
#undef X

const char* map_error_string(MapError e) {
    return map_error_names[e];
}

MapError map_load_from_file(Map* map, const char* file_path) {
    FILE* file = fopen(file_path, "r");
    if (!file) {
        return MAP_ERROR_OPEN_FILE;
    }

    char error_buffer[100];

    toml_table_t* conf = toml_parse_file(file, error_buffer, sizeof(error_buffer));
    if (!conf) {
        return MAP_ERROR_PARSE;
    }

    toml_array_t* obstacles = toml_array_in(conf, "obstacle");
    if (obstacles) {
        int obstacles_array_len = toml_array_nelem(obstacles);

        map->num_obstacles = (size_t) obstacles_array_len;
        map->obstacles = (MapObstacle*) malloc(map->num_obstacles * sizeof(MapObstacle)); 

        for (int i = 0; i < obstacles_array_len; i++) {
            toml_table_t* obstacle_table = toml_table_at(obstacles, i);
            if (!obstacle_table) {
                return MAP_ERROR_FORMAT;    
            }

            toml_array_t* vertex_array = toml_array_in(obstacle_table, "vertices");
            if (!vertex_array) {
                return MAP_ERROR_FORMAT;
            }

            int vertex_array_len = toml_array_nelem(vertex_array);

            if (vertex_array_len % 2 != 0 || vertex_array_len / 2 < 3) {
                return MAP_ERROR_FORMAT;
            }

            map->obstacles[i].num_vertices = (size_t) (vertex_array_len / 2);
            map->obstacles[i].vertices = (float*) malloc(vertex_array_len * sizeof(float));

            for (int j = 0; j < vertex_array_len; j++) {
                const char* raw_vertex = toml_raw_at(vertex_array, j);
                double vertex;
                if (toml_rtod(raw_vertex, &vertex) != 0) {
                    return MAP_ERROR_FORMAT;
                }

                map->obstacles[i].vertices[j] = (float) vertex;
            }
        }

        toml_free(conf);
    } else {
        map->obstacles = NULL;
        map->num_obstacles = 0;
    }

    fclose(file);
    return MAP_ERROR_OK;
}
