import serial


class LLC_Interface:
    def __init__(self, port='/dev/ttyS0', baud=500000):
        self.ser = serial.Serial(port, baud)
        self.ser.flush()

        self.buffer = []

    def __construct_string(self, i, x, y, z):
        return '{},{},{},{}\n'.format(i, x, y, z)

    def add_to_buffer(self, i, x, y, z):
        self.buffer.append(self.__construct_string(i, x, y, z))

    def add_raw(self, val):
        self.buffer.append(f'{val}\n')

    def send_buffer(self):
        for message in self.buffer:
            self.ser.write(message.encode('utf-8'))
        self.buffer = []
