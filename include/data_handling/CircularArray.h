#ifndef CIRCULARARRAY_H
#define CIRCULARARRAY_H

#include <algorithm>
#include <cstdint>
#include <vector>

template<typename T>
int partition(std::vector<T>& array, int left, int right, int pivotIndex) {
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
template<typename T>
T quickSelect(std::vector<T> &array, int left, int right, int k){
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


template<typename T>
class CircularArray {
  protected:
    std::vector<T> array; 
    uint8_t head;       // 0 to 255
    uint8_t maxSize;    // 0 to 255
    uint16_t pushCount; // 0 to 65535

  public:
    CircularArray(uint8_t maxSize){
        this->maxSize = maxSize;
        this->head = 0;
        this->pushCount = 0;
        this->array = std::vector<T>(maxSize, T());
    }

    ~CircularArray(){
        array.clear();
    }

    void push(T data){
        // After the first push, start moving the head
        if (pushCount)
            head = (head + 1) % maxSize;
        array[head] = data;
        pushCount++;
    }

    T pop(){
        if (pushCount == 0){
            return T();
        }
        T data = array[head];
        head = (head + maxSize - 1) % maxSize;
        pushCount--;
        return data;
    }

    // How many indexes back from the head
    T getFromHead(uint8_t index){
        return array[(head + maxSize - index) % maxSize];
    }

    // Has the circular array been filled          
    bool isFull(){
        return pushCount >= maxSize;
    }
    
    bool isEmpty(){
        return pushCount == 0;
    }

    uint8_t getHead(){
        return head;
    }

    uint8_t getMaxSize(){
        return maxSize;
    }

    T getMedian(){
        if (pushCount == 0) {
        // Handle the case when the array is empty
        return T();
        }

        size_t count = std::min(static_cast<size_t>(pushCount), static_cast<size_t>(maxSize));
        std::vector<T> copyArray(count);

        // Collect the valid elements from the circular array
        // TODO: Optimize (or just copy everything and restrict this function to only running when the array is full)
        for (size_t i = 0; i < count; ++i) {
            copyArray[i] = array[(head + maxSize - i) % maxSize];
        }

        // Find the median
        return quickSelect(copyArray, 0, count - 1, count / 2);
    }

    void clear(){
        head = 0;
        pushCount = 0;
        for (int i = 0; i < maxSize; i++){
            array[i] = T();
        }
    }

};



#endif