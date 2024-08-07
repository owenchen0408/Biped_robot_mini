import serial
import time
import threading



class MotorControl:
    def __init__(self, device_name, baudrate, timeout=0):
        self.DEVICE_NAME = device_name
        self.Baud = baudrate
        self.TIME_OUT = timeout
        self.ser = serial.Serial(self.DEVICE_NAME, self.Baud, timeout=timeout)
        self.ser.reset_input_buffer()
        time.sleep(0.01)
        self.ser.reset_output_buffer()
        time.sleep(0.01)
        self.time_ess = 0
        self.command_delay = 1/200
        # self.stop_thread = False
        # self.read_thread = threading.Thread(target=self.read_serial)
        # self.read_thread.start()
        while not self.ser.is_open:
            pass
    
    def calculate_checksum(self, data):
        return sum(data) & 0xFF
    
    def closeport(self):
        self.stop_thread = True
        # self.read_thread.join()
        self.ser.close()

    def clear_buffers(self):
        if self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
    
    def read_serial(self):
        # while not self.stop_thread:
            if self.ser.in_waiting > 0:
                # try:
                #     data = self.ser.read(self.ser.in_waiting)
                #     try:
                #         decoded_data = data.decode('utf-8')
                #         print(f"Received data: {decoded_data}")
                #     except UnicodeDecodeError:
                #         print(f"Received non-UTF-8 data: {data}")
                # except Exception as e:
                #     print(f"Error reading data: {e}")
                try:
                    data=self.ser.read(self.ser.in_waiting)
                    cheaksum = self.calculate_checksum(data[5:12])
                    if cheaksum == data[12]:
                        id = data[2]
                        motortamperature = data[5]
                        motorrpm_raw = (data[9] << 8)| data[8]  # high byte|low byte
                        motorrpm = motorrpm_raw / 360
                        motorrpm_raw = hex(motorrpm_raw)
                        encoder = (data[11] << 8)| data[10] # high byte|low byte  
                        motorinfo = [id, motorrpm, motorrpm_raw,  encoder, motortamperature]
                        return(motorinfo)
                    # return ("id:",id,"Position:",encoder,"rpm:",motorrpm/360,"tamperature:",motortamperature)
                    # else:
                    #     pass

                except Exception as e:
                    return (f"Error reading data: {e}")
            time.sleep(self.command_delay)

    def startmotor(self, id):
        # self.clear_buffers()
        ALLdata = [0x3e, 0x88, id, 0x00]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        self.ser.write(ALLdata)
        time.sleep(self.command_delay)
        self.ser.flush()

    def stopmotor(self, id):
        # self.clear_buffers()
        ALLdata = [0x3e, 0x81, id, 0x00]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        self.ser.write(ALLdata)
        time.sleep(self.command_delay)
        self.ser.flush()

    def readmotorstate(self,id):
        ALLdata = [0x3e,0xa1, id, 0x00]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        self.ser.write(ALLdata)
        time.sleep(self.command_delay)
        self.ser.flush()
        return(self.read_serial())

    def torquecontrol(self, id, torque):
        # self.clear_buffers()
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
        time.sleep(self.command_delay)
        self.ser.flush()
        return(self.read_serial())
        # if self.ser.in_waiting > 0:
        #     readdata = self.ser.read_all()
        #     print(f'ID: {id}    Data: {readdata}')

    def speedcontrol(self, id, speed): #speed unit 0.01dps/

        ALLdata = [0x3e, 0xa2, id, 0x04]
        cmd_sum = self.calculate_checksum(ALLdata)
        ALLdata.append(cmd_sum)
        speed = speed*100
        low_byte = speed & 0xff
        second_byte = (speed >> 8) & 0xff
        third_byte = (speed >> 16) & 0xff
        high_byte = (speed >> 24) & 0xff
        DATA = [low_byte, second_byte, third_byte, high_byte]
        data_sum = self.calculate_checksum(DATA)
        DATA.append(data_sum)
        ALLdata.extend(DATA)
        # ALLdata = [0x3E ,0xA2 ,0x02 ,0x04 ,0xE6 ,0x40 ,0x0D ,0x03 ,0x00 ,0x50]
        self.ser.write(ALLdata)
        time.sleep(self.command_delay)
        # self.ser.flush()
        ALLdata = [f"{num:02x}" for num in ALLdata]
        return(ALLdata,self.read_serial())

def test():

    DEVICE_NAME = '/dev/ttyUSB2'
    BAUDRATE = 115200
    mc = MotorControl(DEVICE_NAME, BAUDRATE)
    
    mc.startmotor(0x01)
    mc.startmotor(0x02)
    time.sleep(1)
    
    try:
        # while True:
        #     for _ in range(250):
        #         print(mc.torquecontrol(0x01, 25))
        #         print(mc.torquecontrol(0x02, 25))

        #     # time.sleep(2)
        #     print('stop\n')
        #     for _ in range(250):
        #         print(mc.torquecontrol(0x01, 0))
        #         print(mc.torquecontrol(0x02, 0))
        #     # time.sleep(2)
        #     print("start\n")

        while True:
            for _ in range(100):
                
                print(mc.speedcontrol(0x01, 1080))
                print(mc.speedcontrol(0x02, 1080))
            # time.sleep(2)
            print('stop\n')
            for _ in range(100):
                print(mc.speedcontrol(0x01, 0))
                print(mc.speedcontrol(0x02, 0))
            # time.sleep(2)
            print("start\n")

    except KeyboardInterrupt:

        mc.stopmotor(0x01)
        mc.stopmotor(0x02)
        mc.closeport()

if __name__ == '__main__':
    test()
