#ifndef ARENA_HPP
#define ARENA_HPP

#include <string.h>
#include <stddef.h>
#include <stdint.h>

template <typename T>
class Stack
{
private:

    int len;
    int cap;
    T* items;

public:

    Stack(int _cap)
    {
        cap = _cap;
        len = 0;

        items = new T[cap];
    }

    ~Stack()
    {
        if (items) delete[] items;
    }

    void push(T t)
    {
        if (len == cap)
        {
            cap = cap * 2;
            T* new_items = new T[cap];
            memcpy(new_items, items, len * sizeof(T));
            delete[] items;
            items = new_items;
        }

        items[len++] = t;
    }

    bool pop(T& t)
    {
        if (len > 0) 
        {
            t = items[len - 1];
            len--;
            return true;
        }

        return false;
    }

};

template <typename T>
struct TContainer {
    int index;
    T t;
};

template <typename T>
class Arena {
private:

    int num_segments;
    int segment_size;
    TContainer<T>** segments;

    Stack<int> indices;

public:

    Arena(int _segment_size) : indices(_segment_size) {
        segment_size = _segment_size;

        segments = new TContainer<T>*[1];

        segments[0] = new TContainer<T>[segment_size];

        for (int i = 0; i < segment_size; i++) {
            indices.push(i);
            segments[0][i].index = i;
        }
    }

    ~Arena() {
        for (int i = 0; i < num_segments; i++) {
            if (segments[i]) delete[] segments[i];
        }

        delete[] segments;
    }

    T* alloc() {
        int index;
        if (!indices.pop(index)) {
            TContainer<T>** new_segments = new TContainer<T>*[num_segments + 1];
            memcpy(new_segments, segments, num_segments*sizeof(TContainer<T>*));
            delete[] segments;
            segments = new_segments;
            num_segments++;
            segments[num_segments - 1] = new TContainer<T>[segment_size];

            for (int i = (num_segments - 1)*segment_size; i < num_segments*segment_size; i++) {
                indices.push(i);
                segments[num_segments - 1][i - (num_segments - 1)*segment_size].index = i;
            }

            indices.pop(index);
        }

        int segment = index / segment_size;
        int segment_offset = index % segment_size;

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