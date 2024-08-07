import serial
import time
import threading

DEVICE_NAME = '/dev/ttyUSB0'
BAUDRATE = 115200

class MotorControl:
    def __init__(self, device_name, baudrate, timeout=0.00):
        self.DEVICE_NAME = device_name
        self.Baud = baudrate
        self.TIME_OUT = timeout
        self.ser = serial.Serial(self.DEVICE_NAME, self.Baud, timeout=timeout)
        self.ser.reset_input_buffer()
        time.sleep(0.01)
        self.ser.reset_output_buffer()
        time.sleep(0.01)
        self.time_ess = 0.0
        self.stop_thread = False
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.start()
        while not self.ser.is_open:
            pass
    
    def calculate_checksum(self, data):
        return sum(data) & 0xFF
    
    def closeport(self):
        self.stop_thread = True
        self.read_thread.join()
        self.ser.close()

    def clear_buffers(self):
        if self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
    
    def read_serial(self):
        while not self.stop_thread:
            if self.ser.in_waiting > 0:
                try:
                    data = self.ser.read(self.ser.in_waiting)
                    try:
                        decoded_data = data.decode('utf-8')
                        print(f"Received data: {decoded_data}")
                    except UnicodeDecodeError:
                        print(f"Received non-UTF-8 data: {data}")
                except Exception as e:
                    print(f"Error reading data: {e}")
            time.sleep(0.1)

    def startmotor(self, id):
        self.clear_buffers()
        ALLdata = [0x3e, 0x88, id, 0x00]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        self.ser.write(ALLdata)

    def stopmotor(self, id):
        self.clear_buffers()
        ALLdata = [0x3e, 0x81, id, 0x00]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        self.ser.write(ALLdata)
        self.ser.flush()
    
    def torquecontrol(self, id, torque):
        self.clear_buffers()
        ALLdata = [0x3e, 0xa1, id, 0x02]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        low_byte = torque & 0xFF
        high_byte = (torque >> 8) & 0xFF
        DATA = [low_byte, high_byte]
        data_sum = self.calculate_checksum(DATA)
        DATA.append(data_sum)
        ALLdata.extend(DATA)
        self.ser.write(ALLdata)
        self.ser.flush()
        # if self.ser.in_waiting > 0:
        #     readdata = self.ser.read_all()
        #     print(f'ID: {id}    Data: {readdata}')

def main():
    mc = MotorControl(DEVICE_NAME, BAUDRATE)
    
    mc.startmotor(0x01)
    mc.startmotor(0x02)
    time.sleep(1)
    while True:
        mc.torquecontrol(0x01, 0)
        # mc.torquecontrol(0x02, 100)
        # time.sleep(2)
        # print('stop\n')
        # mc.torquecontrol(0x01, 0)
        # # mc.torquecontrol(0x02, 0)
        # time.sleep(2)
        # print("start\n")
        time.sleep(1/60)

    # mc.closeport()

if __name__ == '__main__':
    main()
