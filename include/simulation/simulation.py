import serial
import time
import pandas as pd 
import matplotlib.pyplot as plt
import sys 

# Serial port is the first arg
if len(sys.argv) <= 1:
    raise ValueError("Please provide the serial port as the first argument, e.g. 'COM3' or '/dev/ttyACM0'"
    " This is the same port you use to monitor or send commands via the UARTCommandHandler")


serial_port = sys.argv[1]

# Set up the serial connection (adjust the port to your system)
ser = serial.Serial(serial_port, 115200, timeout=1)  # Replace 'COM3' with your port name
time.sleep(2)  # Allow time for the connection to be established

# Clear the buffer
if ser.in_waiting > 0:
    ser.reset_input_buffer()

# Function to wait for the start command
def wait_for_start_command():
    print("Waiting for start command...")
    while True:
        if ser.in_waiting > 0:
            command = ser.readline().strip().decode('utf-8')
            print(command)
            if command == "START":
                # Send ack to the client
                ser.write(b'\x06')
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
    # return f"{target_timestamp},{accelX},{accelY},{accelZ},{gyroX},{gyroY},{gyroZ},{magX},{magY},{magZ},{altitude},{pressure},{temp}\n"

    # Try just sending accel
    return f"{target_timestamp: .3f},{accelX: 3f},{accelY: .3f},{accelZ :.3f},{altitude}\n"

# Function to read data from CSV and stream it over serial with interpolation
def stream_csv_data(csv_file):
    
    # Open the CSV file and save all the data into a 2D list (rows)
    data = pd.read_csv(csv_file)
    rows = data.values.tolist()

    

    first_row_time_ms = rows[0][0]
    start_real_time_ms = time.time() * 1000

    sent_data = [] 

    # Build a map of timestamp to row index
    timestamp_to_row = {}
    for i, row in enumerate(rows):
        timestamp = row[0] - first_row_time_ms
        timestamp_to_row[timestamp] = i

    print("First row time: ", first_row_time_ms)
    print("Start Real time: ", start_real_time_ms)

    current_row = 0

    while True: 
        # Based on how much time has passed since the start_real_time_ms, send the interpolated data
        # from the CSV
        real_elapsed_time_ms = time.time() * 1000 - start_real_time_ms

        # Find the two rows to interpolate between
        # While the timestamp is too small, keep incrementing the row index
        while current_row < len(rows) - 1 and rows[current_row][0] - first_row_time_ms < real_elapsed_time_ms:
            current_row += 1

        prev_row = current_row - 1

        # Check if we have reached the end of the CSV
        if current_row == len(rows) - 1:
            break

        # Interpolate between the two rows
        interpolated_data = interpolate_data(rows[prev_row], rows[current_row], real_elapsed_time_ms + first_row_time_ms)


        # Send the interpolated data over serial
        ser.write(interpolated_data.encode())
        print(f"Sent: {interpolated_data.strip()}", "Current row: ", current_row)

        # Wait for acknowledgment before continuing
        buffer = b''
        wait_for_ack_start_time = time.time()
        while True:
            if ser.in_waiting > 0:
                buffer += ser.read_all()   
                # Check for the series of 0xaa, 0xbb, 0xcc in the buffer to confirm acknowledgment
                if b'\xaa\xbb\xcc' in buffer:
                    # The byte right before the ack series is the state
                    loc = buffer.find(b'\xaa\xbb\xcc')
                    print("Received acknowledgment: ", buffer)
                    state = buffer[loc - 1]
                    break
                else:
                    # pass 
                    print("Waiting for acknowledgment got: ", buffer)
            if time.time() - wait_for_ack_start_time > 5:  # Timeout after 5 seconds
                print("Timeout waiting for acknowledgment, resending data.")
                ser.write(interpolated_data.encode())
                wait_for_ack_start_time = time.time()  # Reset the timer

        sent_data.append(interpolated_data.split(',') + [state])

    print("Done after ", real_elapsed_time_ms, " ms")

    # Graph the sent data
    headers = ['timestamp', 'accelX', 'accelY', 'accelZ', 'altitude', 'state']
    df = pd.DataFrame(sent_data, columns=headers)

    # Convert every column to float
    df = df.astype(float)

    # Normalize timestamps to start at zero and convert to seconds
    df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]
    df['timestamp'] = df['timestamp'] / 1000

    # Detect state changes
    state_changes = df['state'].diff().fillna(0) != 0  # Boolean mask for state changes
    state_change_timestamps = df.loc[state_changes, 'timestamp'].tolist()

    # Create the plot
    plt.figure(figsize=(10, 6))
    plt.plot(df['timestamp'], df['accelX'], label='AccelX')
    plt.plot(df['timestamp'], df['accelY'], label='AccelY')
    plt.plot(df['timestamp'], df['accelZ'], label='AccelZ')
    plt.plot(df['timestamp'], df['altitude'], label='Altitude')

    # Add vertical lines for state changes
    for timestamp in state_change_timestamps:
        plt.axvline(x=timestamp, color='red', linestyle='--', alpha=0.7, label='State Change' if timestamp == state_change_timestamps[0] else "")

    # Labels and legend
    plt.xlabel('Time (s)')
    plt.ylabel('Sensor Values')
    plt.title('Serial Simulation with State Changes')
    plt.legend()
    plt.show()

    # Save the plot as an image
    plt.savefig('serial_simulation_plot.png')

try:
    # Wait for the start command before streaming
    wait_for_start_command()

    # Provide the path to your CSV file here
    csv_file = 'lib/Avionics/include/simulation/AA Data Collection - Second Launch Trimmed.csv'  # Replace with your file path
    stream_csv_data(csv_file)
except KeyboardInterrupt:
    print("Program terminated.")
finally:
    ser.close()  # Close the serial port
