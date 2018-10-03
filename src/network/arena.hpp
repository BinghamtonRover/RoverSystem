#ifndef ARENA_HPP
#define ARENA_HPP

#include <cstring>
#include <cstddef>
#include <cstdint>

#include <stack>

/*
    This file defines an Arena allocator which is used exclusively within the network library.
*/


// A way to store the index of each allocated T.
template <typename T>
struct TContainer {
    int index;
    T t;
};

// An arena allocator.
// Allocates fixed_size segments of Ts on-demand.
// Keeps a stack of available indices.
template <typename T>
class Arena {
private:

    int num_segments;
    int segment_size;
    TContainer<T>** segments;

	std::stack<int> indices;

public:

    Arena(int _segment_size) {
        segment_size = _segment_size;

        // Create the segments array.
        segments = new TContainer<T>*[1];

        // Create the first segment.
        segments[0] = new TContainer<T>[segment_size];

        // Update the indices of the first segment
        // and push them so we know they are available.
        for (int i = 0; i < segment_size; i++) {
            indices.push(i);
            segments[0][i].index = i;
        }
    }

    // Destroys the segments.
    ~Arena() {
        for (int i = 0; i < num_segments; i++) {
            if (segments[i]) delete[] segments[i];
        }

        delete[] segments;
    }

    // Grabs an available T.
    T* alloc() {
        int index;

        if (indices.empty()) {
            // If there are no free indices, then we must create a new segment.
            TContainer<T>** new_segments = new TContainer<T>*[num_segments + 1];
            memcpy(new_segments, segments, num_segments*sizeof(TContainer<T>*));
            delete[] segments;
            segments = new_segments;
            num_segments++;
            segments[num_segments - 1] = new TContainer<T>[segment_size];

            // Set the indices of the new segment.
            // Also push those indices to the stack.
            // Since num_segments was already incremented, we subtrack 1 here. Not the best, not the worst.
            for (int i = (num_segments - 1)*segment_size; i < num_segments*segment_size; i++) {
                indices.push(i);
                // It might be cleaner just to have i go from 0 to segment_size.
                // We subtract that big thing off because its our initial i offset.
                segments[num_segments - 1][i - (num_segments - 1)*segment_size].index = i;
            }
        }

		index = indices.top();
		indices.pop();

        // When we get an index, we need to find the segment to which it belongs.
        // The remainder is the offset within that segment.
        int segment = index / segment_size;
        int segment_offset = index % segment_size;

        // Return a pointer to the value at that offset of that segment.
        return &((segments[segment] + segment_offset)->t);
    }

    void free(T* t) {
        // The TContainer for this T starts before it in memory. We need to backtrack to find that point.
        uint8_t* t_u8 = (uint8_t*)t;
        t_u8 -= offsetof(TContainer<T>, t);
        TContainer<T>* t_container = (TContainer<T>*) t_u8;

        indices.push(t_container->index);
    }

};

#endif
