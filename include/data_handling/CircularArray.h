#ifndef CIRCULARARRAY_H
#define CIRCULARARRAY_H

#include <algorithm>
#include <cstdint>
#include <array>
#include <assert.h>

constexpr std::size_t MAX_CIRCULAR_ARRAY_CAPACITY = 255;

template<typename T, std::size_t N>
int partition(std::array<T, N>& array, int left, int right, int pivotIndex) {
    T pivotValue = array[pivotIndex];
    std::swap(array[pivotIndex], array[right]); // Move pivot to end
    int storeIndex = left;
    for (int i = left; i < right; i++) {
        if (array[i] < pivotValue) {
            std::swap(array[i], array[storeIndex]);
            storeIndex++;
        }
    }
    std::swap(array[storeIndex], array[right]); // Move pivot to its final place
    return storeIndex;
}

// Quick select implementation
// Returns the kth smallest element in the array
// left and right are the bounds of the array
// O(n) average time complexity
// O(n^2) worst case time complexity
// If we were to just use bubble sort, that would be O(n^2) time complexity
template<typename T, std::size_t N>
T quickSelect(std::array<T, N> &array, int left, int right, int k){
    while (left < right){
        int pivotIndex = (left + right) / 2;
        int pivotNewIndex = partition(array, left, right, pivotIndex);
        if (pivotNewIndex == k){
            return array[k];
        } else if (k < pivotNewIndex){
            right = pivotNewIndex - 1;
        } else {
            left = pivotNewIndex + 1;
        }
    }
    return array[k];
}


template<typename T, std::size_t Capacity>
/**
 * @brief Fixed-size circular buffer with median helper and head tracking.
 * @note When to use: maintain a rolling window of recent samples for filters
 *       or detectors without reallocations.
 */
class CircularArray {
  protected:
    std::array<T, Capacity> array; 
    std::array<T, Capacity> scratchArray; // For median calculation
    uint8_t maxSize;    // 0 to 255
    uint8_t head;       // 0 to 255
    uint8_t currentSize; // 0 to 255

  public:
    CircularArray(uint8_t maxSize = Capacity) : maxSize(maxSize) {
        static_assert(Capacity > 0, "CircularArray capacity must be greater than 0");
        static_assert(Capacity <= MAX_CIRCULAR_ARRAY_CAPACITY, "CircularArray capacity must be less than or equal to 255 b/c of head being uint8_t");
        assert(maxSize > 0 && maxSize <= Capacity);
        this->head = 0;
        this->currentSize = 0; // How full is the circular buffer? 
    }

    void push(T data){
        // After the first push, start moving the head
        if (currentSize)
            head = (head + 1) % maxSize;
        array[head] = data;

        // Cap current size at maxSize
        if (currentSize < maxSize){
            currentSize++;
        }
    }

    T pop(){
        if (currentSize == 0){
            return T();
        }
        T data = array[head];
        head = (head + maxSize - 1) % maxSize;
        currentSize--;
        return data;
    }

    // How many indexes back from the head
    T getFromHead(uint8_t index){
        return array[(head + maxSize - index) % maxSize];
    }

    // Has the circular array been filled          
    bool isFull(){
        return currentSize >= maxSize;
    }
    
    bool isEmpty(){
        return currentSize == 0;
    }

    uint8_t getHead(){
        return head;
    }

    uint8_t getMaxSize(){
        return maxSize;
    }

    T getMedian(){
        if (currentSize == 0) {
        // Handle the case when the array is empty
        return T();
        }

        size_t count = std::min(static_cast<size_t>(currentSize), static_cast<size_t>(maxSize));

        // Collect the valid elements from the circular array
        for (size_t i = 0; i < count; ++i) {
            scratchArray[i] = array[(head + maxSize - i) % maxSize];
        }

        // Find the median
        int n = static_cast<int>(count);
        return quickSelect(scratchArray, 0, n - 1, n / 2);
    }

    void clear(){
        head = 0;
        currentSize = 0;
        for (uint8_t i = 0; i < maxSize; i++){
            array[i] = T();
        }
    }

};



#endif