import serial
import struct
from time import sleep

def init():
  test()

def test():
  print('hello')

# Serial setup
ser = serial.Serial(
    port='/dev/ttyAMA0', # Replace with your serial port
    baudrate=9600,
    timeout=1
)


def read_from_arduino():
    # Read a sequence of bytes from Arduino
    try:
        if ser.in_waiting > 0:
            received_data = ser.read(38) # Adjust byte count as needed
            data_tuple = struct.unpack('<19H', received_data)

            rx_data = {
                'RX_SEQ_NUMBER': data_tuple[0],
                'TUBE_PRESS': data_tuple[1],
                'VALVE_STATE': data_tuple[2],
                'PUMP_STATE': data_tuple[3],
                'PUMP_SPD': data_tuple[4],
                'FAN_STATE': data_tuple[5],
                'FAN_SPD': data_tuple[6],
                'IMU_X': data_tuple[7],
                'IMU_Y': data_tuple[8],
                'IMU_Z': data_tuple[9],
                'RADIO_ALT': data_tuple[10],
                'VERT_VELOCITY': data_tuple[11],
                'ADI_PITCH': data_tuple[12],
                'ADI_ROLL': data_tuple[13],
                'GMC_HEADING': data_tuple[14],
                'THROTTLE': data_tuple[15],
                'ENG_RPM': data_tuple[16],
                'TORQUE': data_tuple[17],
                'IAS': data_tuple[18],
                'ROTOR_RPM': data_tuple[19]
            }

            return rx_data
    except serial.SerialException as e:
        print(f"Error reading from serial port: {e}")
    return None

def send_to_arduino(tx_seq_number, set_tube_press):
    # Convert values to bytes and send them to the Arduino
    try:
        # Pack data into bytes
        data = struct.pack('<2H', tx_seq_number, set_tube_press)
        # Write packed bytes to the serial port
        ser.write(data)
    except serial.SerialException as e:
        print(f"Error writing to serial port: {e}")

def main():
    while True:
        # Example sending data to Arduino
        send_to_arduino(1, 100) # Replace with dynamic data as needed

        # Read data from Arduino
        arduino_data = read_from_arduino()
        if arduino_data is not None:
            print("Received data from Arduino:", arduino_data)

        sleep(1) # Delay for a second

if __name__ == "__main__":
    main()