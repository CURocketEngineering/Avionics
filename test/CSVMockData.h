#ifndef CSV_MOCK_DATA_H
#define CSV_MOCK_DATA_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include <algorithm>

// Linear interpolation helper function
inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

// Structure to hold sensor data parsed from the CSV
struct SensorData {
    long time;  // Time in milliseconds
    float accelx;
    float accely;
    float accelz;
    float gyrox;
    float gyroy;
    float gyroz;
    float magx;
    float magy;
    float magz;
    float altitude;
    float pressure;
    float temp;
};

// Helper function to parse a row of CSV data
inline SensorData parseCSVRow(const std::vector<std::string>& row) {
    if (row.size() < 13) {
        std::cerr << "Invalid row data: expected 13 columns, got " << row.size() << std::endl;
        exit(EXIT_FAILURE);
    }
    SensorData data;
    data.time    = stoll(row[0]);
    data.accelx  = stof(row[1]);
    data.accely  = stof(row[2]);
    data.accelz  = stof(row[3]);
    data.gyrox   = stof(row[4]);
    data.gyroy   = stof(row[5]);
    data.gyroz   = stof(row[6]);
    data.magx    = stof(row[7]);
    data.magy    = stof(row[8]);
    data.magz    = stof(row[9]);
    data.altitude= stof(row[10]);
    data.pressure= stof(row[11]);
    data.temp    = stof(row[12]);
    return data;
}

// CSVDataProvider class reads and parses the CSV file.
class CSVDataProvider {
private:
    std::ifstream file;
    std::string currentLine;
    std::vector<SensorData> allData;
    bool dataLoaded;
    float sampleRate_hz;
    long currentTime_ms;  // Current time in milliseconds
    long timeStep_ms;     // Time step in milliseconds

    void loadAllData() {
        while (static_cast<bool>(getline(file, currentLine))) {
            std::vector<std::string> row;
            std::stringstream ss(currentLine);
            std::string cell;
            while (getline(ss, cell, ',')) {
                row.push_back(cell);
            }
            if (row.size() >= 13) {
                allData.push_back(parseCSVRow(row));
            }
        }
        std::sort(allData.begin(), allData.end(), 
                 [](const SensorData& a, const SensorData& b) { 
                     return a.time < b.time; 
                 });
        dataLoaded = true;

        // Print debug info about loaded data
        if (!allData.empty()) {
            std::cout << "Loaded " << allData.size() << " data points\n";
            std::cout << "Time range: " << allData.front().time << " to " << allData.back().time << " ms\n";
            if (sampleRate_hz > 0) {
                long total_time = allData.back().time - allData.front().time;
                long expected_samples = total_time / timeStep_ms;
                std::cout << "Sample rate: " << sampleRate_hz << " Hz (step: " << timeStep_ms << " ms)\n";
                std::cout << "Expected samples at " << sampleRate_hz << " Hz: " << expected_samples << "\n";
            }
        }
    }

public:
    CSVDataProvider(const std::string& filename, float sampleRate_hz = 0) : 
        dataLoaded(false),
        sampleRate_hz(sampleRate_hz),
        currentTime_ms(0),
        timeStep_ms(sampleRate_hz > 0 ? static_cast<long>((1.0 / sampleRate_hz) * 1000.0) : 0) {
        file.open(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open CSV file: " << filename << std::endl;
            exit(EXIT_FAILURE);
        }
        // Skip header line
        getline(file, currentLine);
        loadAllData();
    }

    // Get interpolated data at a specific timestamp
    SensorData getInterpolatedData(long timestamp) {
        if (!dataLoaded || allData.empty()) {
            std::cerr << "No data available for interpolation" << std::endl;
            exit(EXIT_FAILURE);
        }

        // Find the two data points to interpolate between
        auto it = std::lower_bound(allData.begin(), allData.end(), timestamp,
                                 [](const SensorData& data, long ts) {
                                     return data.time < ts;
                                 });

        // Handle edge cases
        if (it == allData.begin()) {
            return allData.front();
        }
        if (it == allData.end()) {
            return allData.back();
        }

        // Get the two points to interpolate between
        const SensorData& next = *it;
        const SensorData& prev = *(--it);

        // Calculate interpolation factor
        float t = static_cast<float>(timestamp - prev.time) / 
                 static_cast<float>(next.time - prev.time);

        // Create interpolated data point
        SensorData result;
        result.time = timestamp;
        result.accelx = lerp(prev.accelx, next.accelx, t);
        result.accely = lerp(prev.accely, next.accely, t);
        result.accelz = lerp(prev.accelz, next.accelz, t);
        result.gyrox = lerp(prev.gyrox, next.gyrox, t);
        result.gyroy = lerp(prev.gyroy, next.gyroy, t);
        result.gyroz = lerp(prev.gyroz, next.gyroz, t);
        result.magx = lerp(prev.magx, next.magx, t);
        result.magy = lerp(prev.magy, next.magy, t);
        result.magz = lerp(prev.magz, next.magz, t);
        result.altitude = lerp(prev.altitude, next.altitude, t);
        result.pressure = lerp(prev.pressure, next.pressure, t);
        result.temp = lerp(prev.temp, next.temp, t);

        return result;
    }

    // Get the time range of the data
    std::pair<long, long> getTimeRange() const {
        if (!dataLoaded || allData.empty()) {
            return {0, 0};
        }
        return {allData.front().time, allData.back().time};
    }

    // Get next data point at the specified sample rate
    SensorData getNextDataPoint() {
        if (!dataLoaded || allData.empty()) {
            std::cerr << "No data available" << std::endl;
            exit(EXIT_FAILURE);
        }

        // If no sample rate specified, return raw data points
        if (sampleRate_hz <= 0) {
            if (!hasNextRaw()) {
                std::cerr << "No more raw data points available" << std::endl;
                exit(EXIT_FAILURE);
            }
            return parseCSVRow(getRawRow());
        }

        // Initialize current time if needed
        if (currentTime_ms == 0) {
            currentTime_ms = allData.front().time;
        }

        // Get interpolated data at the current time
        SensorData data = getInterpolatedData(currentTime_ms);
        
        // Debug output every 1000 samples
        static int sample_count = 0;
        if (++sample_count % 1000 == 0) {
            // std::cout << "Processing time: " << currentTime_ms << " ms\n";
        }
        
        // Advance to next sample time
        currentTime_ms += timeStep_ms;

        return data;
    }

    // Check if more data points are available
    bool hasNextDataPoint() {
        if (!dataLoaded || allData.empty()) {
            return false;
        }
        
        // If no sample rate specified, check for raw data
        if (sampleRate_hz <= 0) {
            return hasNextRaw();
        }

        // For interpolated data, check if we can get one more sample
        // We need at least one more timeStep_ms worth of data
        bool hasNext = currentTime_ms < allData.back().time;
        if (!hasNext) {
            std::cout << "Reached end of data at time: " << currentTime_ms << " ms\n";
            std::cout << "Last data point time: " << allData.back().time << " ms\n";
        }
        return hasNext;
    }

    // Raw data access methods (renamed from original hasNext/getRow)
    bool hasNextRaw() {
        return static_cast<bool>(getline(file, currentLine));
    }

    std::vector<std::string> getRawRow() {
        std::vector<std::string> row;
        std::stringstream ss(currentLine);
        std::string cell;
        while (getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        return row;
    }

    // Set or update the sample rate
    void setSampleRate(float hz) {
        sampleRate_hz = hz;
        timeStep_ms = hz > 0 ? static_cast<long>((1.0 / hz) * 1000.0) : 0;
        // Reset current time to start fresh with new rate
        currentTime_ms = 0;
    }

    // Get current sample rate
    float getSampleRate() const {
        return sampleRate_hz;
    }
};

#endif // CSV_MOCK_DATA_H
