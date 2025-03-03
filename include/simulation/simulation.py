import serial
import time
import csv

# Set up the serial connection (adjust the port to your system)
ser = serial.Serial('/dev/tty.usbserial-B003B38N', 115200, timeout=1)  # Replace 'COM3' with your port name
time.sleep(2)  # Allow time for the connection to be established

# Function to wait for the start command
def wait_for_start_command():
    print("Waiting for start command...")
    while True:
        if ser.in_waiting > 0:
            command = ser.readline().strip().decode('utf-8')
            if command == "START":
                print("Received start command, beginning data transmission.")
                break

# Function to interpolate between two data rows
def interpolate_data(row1, row2, target_timestamp):
    timestamp1, accelX1, accelY1, accelZ1, gyroX1, gyroY1, gyroZ1, magX1, magY1, magZ1, altitude1, pressure1, temp1 = map(float, row1)
    timestamp2, accelX2, accelY2, accelZ2, gyroX2, gyroY2, gyroZ2, magX2, magY2, magZ2, altitude2, pressure2, temp2 = map(float, row2)

    # Calculate the interpolation factor
    t1 = timestamp1
    t2 = timestamp2
    alpha = (target_timestamp - t1) / (t2 - t1)

    # Interpolate each data point
    accelX = accelX1 + alpha * (accelX2 - accelX1)
    accelY = accelY1 + alpha * (accelY2 - accelY1)
    accelZ = accelZ1 + alpha * (accelZ2 - accelZ1)
    gyroX = gyroX1 + alpha * (gyroX2 - gyroX1)
    gyroY = gyroY1 + alpha * (gyroY2 - gyroY1)
    gyroZ = gyroZ1 + alpha * (gyroZ2 - gyroZ1)
    magX = magX1 + alpha * (magX2 - magX1)
    magY = magY1 + alpha * (magY2 - magY1)
    magZ = magZ1 + alpha * (magZ2 - magZ1)
    altitude = altitude1 + alpha * (altitude2 - altitude1)
    pressure = pressure1 + alpha * (pressure2 - pressure1)
    temp = temp1 + alpha * (temp2 - temp1)

    # Return the interpolated data as a string
    return f"{target_timestamp},{accelX},{accelY},{accelZ},{gyroX},{gyroY},{gyroZ},{magX},{magY},{magZ},{altitude},{pressure},{temp}\n"

# Function to read data from CSV and stream it over serial with interpolation
def stream_csv_data(csv_file):
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row if it exists

        prev_row = next(reader)  # Get the first row
        prev_timestamp = float(prev_row[0])

        for current_row in reader:
            current_timestamp = float(current_row[0])

            # Interpolate based on the current timestamp and send the data
            interpolated_data = interpolate_data(prev_row, current_row, current_timestamp)
            ser.write(interpolated_data.encode())  
            print(f"Sent interpolated data: {interpolated_data.strip()}")

            # Wait for acknowledgment before continuing
            while True:
                if ser.in_waiting > 0:
                    ack = ser.read(1)  # Read 1 byte for acknowledgment
                    if ack == b'A':  # Assuming STM32 sends 'A' for acknowledgment
                        print("Received acknowledgment, continuing...")
                        break
                    else:
                        print("Waiting for acknowledgment...")

            prev_row = current_row  # Update previous row
            prev_timestamp = current_timestamp  # Update previous timestamp

try:
    # Wait for the start command before streaming
    wait_for_start_command()

    # Provide the path to your CSV file here
    csv_file = 'AA Data Collection - Second Launch Trimmed.csv'  # Replace with your file path
    stream_csv_data(csv_file)
except KeyboardInterrupt:
    print("Program terminated.")
finally:
    ser.close()  # Close the serial port
