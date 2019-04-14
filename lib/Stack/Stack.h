/*
 * Defines a templated (generic) class for a stack of things.
 * 
 */

#ifndef __STACK_H__
#define __STACK_H__

// #include <Arduino.h>

template <class T>
class Stack
{
private:
    int _count;
    T *_data;
    int _maxitems;

public:
    Stack(int maxitems = 256)
    {
        _count = 0;
        _maxitems = maxitems;
        _data = new T[maxitems + 1];
    }
    ~Stack()
    {
        delete[] _data;
    }
    inline int count();
    void push(const T &item);
    T peek();
    T pop();
    void clear();
};

template <class T>
inline int Stack<T>::count()
{
    return _count;
}

template <class T>
void Stack<T>::push(const T &item)
{
    if (_count < _maxitems)
    { // Drops out when full
        _data[_count - 1] = item;
        ++_count;
    }
}

template <class T>
T Stack<T>::pop()
{
    if (_count <= 0)
        return T(); // Returns empty
    else
    {
        T result = _data[_count - 1];
        --_count;
        // Check wrap around
        return result;
    }
}

template <class T>
T Stack<T>::peek()
{
    if (_count <= 0)
        return T(); // Returns empty
    else
        return _data[_count - 1];
}

template <class T>
void Stack<T>::clear()
{
    _count = 0;
}

#endif //__STACK_H__