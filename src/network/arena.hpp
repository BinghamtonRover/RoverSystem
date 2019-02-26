#ifndef ARENA_HPP
#define ARENA_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#include <stack>
#include <vector>

const int ARENA_NUM_THINGS = 10;

/*
    This file defines an Arena allocator which is used exclusively within the network library.
*/

// A way to store the index of each allocated T.
template <typename T> struct TContainer
{
    int index;
    T t;
};

// An arena allocator.
// Allocates fixed_size segments of Ts on-demand.
// Keeps a stack of available indices.
template <typename T> class Arena
{
private:
	TContainer<T> things[ARENA_NUM_THINGS];

    std::stack<int> indices;

public:
    Arena(int _segment_size)
    {
        for (int i = 0; i < ARENA_NUM_THINGS; i++) {
            indices.push(i);
			things[i].index = i;
        }
    }

    // Grabs an available T.
    T *alloc()
    {
        if (indices.empty()) {
			fprintf(stderr, "[!] [!] [!] OUT OF THINGS!\n");
			exit(2);
        }

        int index = indices.top();
        indices.pop();

        return &(things[index].t);
    }

    void free(T *t)
    {
        // The TContainer for this T starts before it in memory. We need to backtrack to find that point.
        uint8_t *t_u8 = (uint8_t *)t;
        t_u8 -= offsetof(TContainer<T>, t);
        TContainer<T> *t_container = (TContainer<T> *)t_u8;

        indices.push(t_container->index);
    }
};

#endif
