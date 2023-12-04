import serial
import struct
import csv
import time
import signal
import sys

csv_filename = "data1.csv" 
def decode_hex_data(hex_line):
    # Remove the 'b' prefix and newline character, then convert to a bytes object
    hex_bytes = bytes.fromhex(hex_line.decode('ascii').rstrip())

    # Unpack binary data into three 16-bit integers (signed shorts)
    short_values = struct.unpack('<hhh', hex_bytes)

    return short_values

def read_serial_binary(port, baudrate=115200, timeout=1, inter_byte_timeout=0.01):
    try:
        # Open the serial port
        ser = serial.Serial(port, baudrate, timeout=1)

        # Variables for tracking lines per second and last received line
        lines_received = 0
        start_time = time.time()
        prev_values = (-11, -7, -6)
       # Open CSV file for writing
        with open(csv_filename, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['AccelX', 'AccelY', 'AccelZ'])  # Write header
            def signal_handler(signal, frame):
                nonlocal ser, csv_file
                print("Ctrl+C pressed. Stopping the script...")
                ser.close()
                csv_file.close()
                sys.exit(0)
            signal.signal(signal.SIGINT, signal_handler)
            while True:
                # Read binary data (6 bytes in this case)
                binary_data = ser.readline()
                values = decode_hex_data(binary_data)

                if values != prev_values:
                    prev_values = values
                    # lines_received += 1

                    # Write to CSV
                    csv_writer.writerow(values)
    finally:
        # Close the serial port
        if ser.is_open:
            ser.close()
        if 'csv_file' in locals() and not csv_file.closed:
            csv_file.close()

if __name__ == "__main__":
    # Specify the COM port and baud rate
    com_port = 'COM22'
    baud_rate = 115200

    # Read binary data from COM22 and convert to integers
    read_serial_binary(com_port, baud_rate)
