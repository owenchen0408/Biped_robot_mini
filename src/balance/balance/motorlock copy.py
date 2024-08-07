import time
from module.DXL_motor_control import DXL_Conmunication

dxl = DXL_Conmunication(device_name="/dev/dxl", b_rate=57600)


def disableALLmotor():

    dxl.disableAllMotor()
    dxl.closeHandler()


def main():

    motor01 = dxl.createMotor("motor01", 1)
    
    dxl.addAllBuckPrarmeter()
    motor01.switchMode('position')

    motor01.enableMotor()

    dxl.updateMotorData()
    time.sleep(2)
    motor01.writePosition(3000)  # 2760-1795

    dxl.sentAllCmd()

    try:

        while True:
            pass

    except KeyboardInterrupt:

        disableALLmotor()


if __name__ == '__main__':

    main()
