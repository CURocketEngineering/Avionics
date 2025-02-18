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

# Function to read data from CSV and stream it over serial
def stream_csv_data(csv_file):
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row if it exists
        for row in reader:
            # Ensure the row has all the necessary data points
            if len(row) == 13:
                timestamp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ, altitude, pressure, temp = row

                # Prepare the formatted data string
                data_str = f"{timestamp},{accelX},{accelY},{accelZ},{gyroX},{gyroY},{gyroZ},{magX},{magY},{magZ},{altitude},{pressure},{temp}\n"
                # Send the data via UART
                ser.write(data_str.encode())  
                print(f"Sent data: {data_str.strip()}")  # Optional: Print data being sent
                
                # Wait for the acknowledgment from STM32 before continuing
                while True:
                    if ser.in_waiting > 0:
                        ack = ser.read(1)  # Read 1 byte for acknowledgment
                        if ack == b'A':  # Assuming STM32 sends 'A' for acknowledgment
                            print("Received acknowledgment, sending next data...")
                            break
                        else:
                            print("Waiting for acknowledgment...")

                # time.sleep(0.1)  # Delay between sending each line of data (can adjust this)

try:
    # Wait for the start command before streaming
    wait_for_start_command()

    # Provide the path to your CSV file here
    csv_file = 'AA Data Collection - Second Launch(in).csv'  # Replace with your file path
    stream_csv_data(csv_file)
except KeyboardInterrupt:
    print("Program terminated.")
finally:
    ser.close()  # Close the serial port
