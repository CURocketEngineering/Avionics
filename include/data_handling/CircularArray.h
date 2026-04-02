#ifndef CIRCULARARRAY_H
#define CIRCULARARRAY_H

#include <algorithm>
#include <array>
#include <assert.h>
#include <cstdint>

constexpr std::size_t kMaxCircularArrayCapacity = 255;

template<typename T, std::size_t N>
std::size_t partition(std::array<T, N>& array, std::size_t left, std::size_t right, std::size_t pivotIndex) {
    T pivotValue = array[pivotIndex];
    std::swap(array[pivotIndex], array[right]); // Move pivot to end
    std::size_t storeIndex = left;
    for (std::size_t i = left; i < right; ++i) {
        if (array[i] < pivotValue) {
            std::swap(array[i], array[storeIndex]);
            ++storeIndex;
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
T quickSelect(std::array<T, N>& array, std::size_t left, std::size_t right, std::size_t k) {
    while (left < right){
        const std::size_t pivotIndex = left + (right - left) / 2;
        const std::size_t pivotNewIndex = partition(array, left, right, pivotIndex);
        if (pivotNewIndex == k){
            return array[k];
        } else if (k < pivotNewIndex){
            if (pivotNewIndex == 0U) {
                break;
            }
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
    CircularArray(uint8_t maxSize_in = static_cast<uint8_t>(Capacity)) : maxSize(maxSize_in) {
        static_assert(Capacity > 0, "CircularArray capacity must be greater than 0");
        static_assert(Capacity <= kMaxCircularArrayCapacity, "CircularArray capacity must be less than or equal to 255 b/c of head being uint8_t");
        assert(maxSize_in > 0 && maxSize_in <= Capacity);
        this->head = 0;
        this->currentSize = 0; // How full is the circular buffer? 
    }

    void push(T data){
        // After the first push, start moving the head
        if (currentSize) {
            const uint16_t nextHead = static_cast<uint16_t>(head) + 1U;
            head = static_cast<uint8_t>(nextHead % maxSize);
        }
        array[static_cast<std::size_t>(head)] = data;

        // Cap current size at maxSize
        if (currentSize < maxSize){
            currentSize++;
        }
    }

    T pop(){
        if (currentSize == 0){
            return T();
        }
        T data = array[static_cast<std::size_t>(head)];
        const uint16_t headStep = static_cast<uint16_t>(head) + static_cast<uint16_t>(maxSize) - 1U;
        head = static_cast<uint8_t>(headStep % maxSize);
        currentSize--;
        return data;
    }

    // How many indexes back from the head
    T getFromHead(uint8_t index){
        const std::size_t headIndex = static_cast<std::size_t>(head);
        const std::size_t maxSizeIndex = static_cast<std::size_t>(maxSize);
        const std::size_t offset = static_cast<std::size_t>(index);
        return array[(headIndex + maxSizeIndex - offset) % maxSizeIndex];
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
        return quickSelect(scratchArray, 0U, count - 1U, count / 2U);
    }

    void clear(){
        head = 0;
        currentSize = 0;
        for (uint8_t i = 0; i < maxSize; i++){
            array[static_cast<std::size_t>(i)] = T();
        }
    }

};



#endif
