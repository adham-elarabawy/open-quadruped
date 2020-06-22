import serial


class LLC_Interface:
    def __init__(self, port='/dev/ttyS0', baud=115200):
        self.ser = serial.Serial(port, baud)
        self.ser.flush()

        self.buffer = []

    def __construct_string(self, i, x, y, z):
        return f'{i},{desired_x},{desired_y},{desired_z}\n'

    def add_to_buffer(self, i, x, y, z):
        buffer.append(self.__construct_string(i, x, y, z)

    def send_buffer():
        for message in buffer:
            ser.write(message.encode('utf-8'))
        self.buffer=[]
