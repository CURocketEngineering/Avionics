# Telemetry

The telemetry system allows the flight computer to send data over a UART stream to a radio modem which transmits the data OTA in real time to the ground station. From the flight computer software standpoint, we are simply sending data over a serial line.

## Lifecycle

Once the flight computer is powered on, the Telemetry object is constructed with the desired SensorDataHandler objects and their corresponding send frequencies. Every loop, the tick() method should be called, which checks if it is time to send data, the format of which is discussed below.

## Packet structure

**All specific signal bytes and lengths are defined in the  TelemetryFmt namespace and may change after the time of writing this documentation.**

**All bytes are in big endian format.**

| Sync | Timestamp | Data... | End
|------|--------|---------|---|
| 4 bytes  | int32     | ... | 4 bytes

 - The first part of a packet is the sync part, which allows the ground station computer to know when a packet begins.
 - The timestamp is the return value of millis() as it is passed when tick() is called.
 - Data (format is discussed below)
 - End marker, which has a similar structure to the sync part.

Within the data part of the packet, there are two ways to format data. 

### Single number data format

If the data simply consists of one number (eg. altitude reading, battery voltage), then it is sent like this:

| Label | Data |
|------|--------|
| 1 byte | 4 bytes

for a total of 5 bytes, where the label is a data label as defined in DataNames.h and the data is simply the big endian byte description of the data.

### Multi number data format

If the data intuitively comes in groups (eg. accelerometer x, y, and z readings), then it is sent like this:

| Label | Value1 | Value2 | Value3 | 
|------|--------|--------|--------|
| 1 byte | 4 bytes | 4 bytes | 4 bytes

 - There is no limit to the amount of numbers you can send per group.
 - The labels for these groups are not present in DataNames.h, they have their own convention, which at the time of this writing is (100 + label number of first number in group). For example, ACCELEROMETER_X is 0 so the accelerometer group label would be 100.

## Practical implementation notes

- The RFD that we used at the time of this writing has an AT mode than can be activated by sending it '+++' over the Serial line. Therefore, do NOT make the sync or end markers "43 43 43". More information on the AT mode of the RFD can be found [here](https://files.rfdesign.com.au/Files/documents/RFD900x%20DataSheet%20V1.2.pdf).
- The RFD and any other radio we may use in the future that acts like a Serial line does not have a true packet capacity. This is because the RFD modem will break up a lot of serial data sent in a short time into packets based on its own transport protocol Therefore, the kPacketCapacity parameter in TelemetryFmt should be set to the amount that the radio datasheet claims it will send it one packet.
