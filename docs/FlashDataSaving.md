# Restricted Flash Storage Specifications

**The data examples shown below are relative to the MARTHA 1.3 hardware.**

## Variable Data Rates  

Not all sensor data needs to be stored at 100Hz. High-frequency data such as acceleration and gyroscope readings are critical for accurate calculations, while other values like temperature and magnetometer readings change more slowly and can be logged less frequently.  

### **Data Save Rates (No Labels)**  

| Data Stream       | Bytes per Save | Saves per Second (Hz) | Total Bytes per Second |
|-------------------|---------------|-----------------------|------------------------|
| Altitude         | 4             | 100                   | 400                    |
| Acceleration (x, y, z) | 12       | 100                   | 1200                   |
| Gyroscope (x, y, z)   | 12       | 100                   | 1200                   |
| Temperature      | 4             | 1                     | 4                      |
| Magnetometer (x, y, z) | 12      | 1                     | 12                     |
| Flight Status    | 4             | 10                    | 40                     |
| Timestamp        | 4             | 100                   | 400                    |
| Super Loop Rate  | 4             | 1                     | 4                      |
| Flight ID        | 4             | 1                     | 4                      |
| **Total**        | -             | -                     | **3264** bytes/sec      |

## **Maximum Storage Capacity**  

The W25Q128 flash chip provides **128 megabits (16 megabytes)** of storage. At **3,264 bytes per second**, this allows for **4,901 seconds (~81 minutes) of storage**, assuming perfect efficiency (which is unrealistic due to metadata overhead).  

## **Metadata Storage**  

The flash chip consists of **4,096 sectors**, with each sector spanning **4,096 addresses** (16³). We reserve **Sector 1** for metadata, as data can only be erased one sector at a time.  

### **Metadata Layout**  

| Address Range | Function |
|--------------|----------|
| `0x000000` (1 byte) | `0x01`: Post-launch mode active, `0x00`: Normal mode |
| `0x000001 – 0x000005` (4 bytes) | Stores the address of "sacred" data (protected launch data) |

Excluding metadata, **15.9959MB** remains available, reducing storage by only **~1 second**.  

## **Writing Strategy**  

### **Sensor Data Handling**  

Each sensor data handler writes to a global data saver that manages flash storage. Each handler can configure its **save rate** based on the table in Fig. 1.  

- **Each data point requires 5 bytes**:  
  - **1 byte**: Label  
  - **4 bytes**: Data (`float` or `uint32_t`)  
- **Example**: Z-acceleration (`0x02` label)  

### **Byte5 Data Format**  

The **Byte5 format** ensures flexible and efficient storage:  
- **1st byte**: `uint8_t` enum (identifies data type)  
- **Next 4 bytes**: `float` or `uint32_t`  

To optimize performance, data is **buffered in RAM** and written **one full 256-byte page at a time**, reducing write cycles.  

### **Byte5 Data Breakdown (Per 256-Byte Page)**  

| Data | Labels | Padding | Bytes per Page |
|------|--------|---------|---------------|
| 204  | 51     | 1       | 256           |

This results in **20.3% storage waste**, balancing efficiency and flexibility.  

### **Data Save Rates (Byte5 Format)**  

| Data Stream       | Bytes per Save | Saves per Second (Hz) | Total Bytes per Second |
|-------------------|---------------|-----------------------|------------------------|
| Altitude         | 5             | 100                   | 500                    |
| Acceleration (x, y, z) | 15      | 100                   | 1500                   |
| Gyroscope (x, y, z)   | 15      | 100                   | 1500                   |
| Temperature      | 5             | 1                     | 5                      |
| Magnetometer (x, y, z) | 15      | 1                     | 15                     |
| Flight Status    | 5             | 10                    | 50                     |
| Timestamp        | 5             | 100                   | 500                    |
| Super Loop Rate  | 5             | 1                     | 5                      |
| Flight ID        | 5             | 1                     | 5                      |
| **Total**        | -             | -                     | **4080** bytes/sec      |

At **4,080 bytes/sec**, storage lasts **3,921 seconds (~65 minutes)**—sufficient for a **5-minute launch**.  

## **Alternative Approach: 64-Byte Chunks**  

A **64-byte chunk method** was considered to eliminate padding and labels by storing fixed-size blocks. This would perfectly align with **256-byte flash pages**.  

### **Proposed 64-Byte Data Chunk**  

| Data Type                     | Bytes |
|--------------------------------|-------|
| Acceleration (x, y, z)        | 12    |
| Gyroscope (x, y, z)           | 12    |
| Magnetometer (x, y, z)        | 12    |
| Altitude                      | 4     |
| Pressure                      | 8     |
| Temperature (LSM6DSOX)        | 4     |
| Temperature (BMP390)          | 4     |
| Timestamp                     | 4     |
| Flag                          | 4     |
| **Total**                     | **64** |

### **Why We Rejected the 64-Byte Chunk Method**  

1. **Forced Uniform Save Rates**  
   - Data that doesn’t change frequently (e.g., temperature, magnetometer) is saved at **100Hz**, **wasting 24 bytes per chunk** (or **96 bytes per page**).  
   - This results in **37.5% storage waste**, compared to **20.3% with Byte5**.  

2. **No Flexibility for New Metrics**  
   - Cannot add new debug data without removing an existing field.  

3. **Byte5 Allows Dynamic Data Logging**  
   - Byte5 enables **variable save rates**, reducing wasted space.  
   - If we **later need** 64-byte chunks at 100Hz, we can revisit this method.  

## **Post-Launch Data Protection**  

To avoid losing launch data due to circular buffering, we implement a **post-launch mode**:

1. **Launch Detection**  
   - When launch occurs, the system **marks the current data address** as the start of **"sacred" data**.  

2. **Protecting Critical Post-Launch Data**  
   - New data **cannot overwrite launch data**. Only the **next hour of data** is recorded.  

3. **Metadata Persistence**  
   - A **post-launch mode flag** is stored in **metadata** to prevent overwriting on reboot.  

### **Summary**  
- **Byte5 format** provides **flexibility & reduced waste**.  
- **64-byte chunks** force unnecessary data logging and are **less adaptable**.  
- **Post-launch mode** ensures we retain the most valuable flight data.  

