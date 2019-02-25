#ifndef ARENA_HPP
#define ARENA_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>

#include <stack>
#include <vector>

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
    int segment_size;
    // `segments` is a vector of pointers to segments of memory used for allocation.
    std::vector<TContainer<T> *> segments;

    std::stack<int> indices;

public:
    Arena(int _segment_size) : segments(), indices()
    {
        segment_size = _segment_size;

        // Create the first segment.
        segments.push_back(new TContainer<T>[segment_size]);


        // Update the indices of the first segment
        // and push them so we know they are available.
        for (int i = 0; i < segment_size; i++) {
            indices.push(i);
            segments[0][i].index = i;
        }
    }

    // Destroys the segments.
    ~Arena()
    {
        for (size_t i = 0; i < segments.size(); i++) {
            delete[] segments[i];
        }
    }

    // Grabs an available T.
    T *alloc()
    {
        int index;

        if (indices.empty()) {
            // If there are no free indices, then we must create a new segment.
            segments.push_back(new TContainer<T>[segment_size]);

            // Set the indices of the new segment.
            // Also push those indices to the stack.
            for (int i = 0; i < segment_size; i++) {
                // Convert to a "global" index (from the first segment).
				int idx = (segments.size() - 1) * segment_size + i;
                indices.push(idx);

                segments[segments.size() - 1][i].index = idx;
            }
        }

        index = indices.top();
        indices.pop();


        // When we get an index, we need to find the segment to which it belongs.
        // The remainder is the offset within that segment.
        int segment = index / segment_size;
        int segment_offset = index % segment_size;


		memset(&(segments[segment][segment_offset].t), 0, sizeof(T));

        // Return a pointer to the value at that offset of that segment.
        return &(segments[segment][segment_offset].t);
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
