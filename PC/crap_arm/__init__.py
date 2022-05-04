from enum import Enum
import struct
import serial


class Op(Enum):
    Home = 0x01
    Move = 0x02
    Beep = 0x03
    Unlock = 0x04
    MoveAngles = 0x05
    St = 0x40
    StIk = 0x41
    Ping = 0x7f
    ReadAngle = 0x80
    ReadPos = 0x81


class Ack(Enum):
    Nack = 0x01
    Ack = 0x02
    Msg = 0xff


class Nack(Enum):
    Op = 0xff
    Sum = 0xfe
    Arg = 0xfd
    NoGoal = 0xfc


class NACKException(Exception):
    pass


class CRAP:
    BYTESIZE = 8
    PARITY = "N"
    STOPBITS = 1

    def __init__(self, com="COM3", baudrate=9600, msg_handler=None):
        self.ser = serial.Serial(
            com,
            baudrate=baudrate,
            bytesize=self.BYTESIZE,
            parity=self.PARITY,
            stopbits=self.STOPBITS
        )

        self.msg_handler = msg_handler

    def send(self, op, data=b""):
        if isinstance(op, Op):
            op = op.value
        pkt = bytes(bytearray([op, len(data), *data, sum(data) & 0xff]))
        self.ser.write(pkt)

        while (ack := self.ser.read(1)[0]) == Ack.Msg.value:
            self.recv_msg()

        if ack != Ack.Ack.value:
            raise NACKException(Nack(self.ser.read(1)[0]))

        if op & 0x80:
            resp_len = self.ser.read(1)[0]
            return self.ser.read(resp_len)

        return None

    def recv_msg(self):
        msg_len = self.ser.read(1)[0]
        msg = self.ser.read(msg_len)
        print("M:" + msg.decode("latin-1"), end="")
        if self.msg_handler is not None:
            self.msg_handler(msg.decode("latin-1"))

    def ping(self):
        return self.send(Op.Ping)

    def home(self):
        return self.send(Op.Home)

    def beep(self):
        return self.send(Op.Beep)

    def unlock(self):
        return self.send(Op.Unlock)

    def move(self, x, y, z):
        return self.send(Op.Move, struct.pack("3d", x, y, z))

    def move_a(self, a0, a1, a2, a3, a4, speed=1000):
        return self.send(Op.MoveAngles, struct.pack("5di", a0, a1, a2, a3, a4, speed))

    def selfcheck(self):
        return self.send(Op.St)

    def selfcheck_ik(self):
        return self.send(Op.StIk)

    def get_angles(self):
        return struct.unpack("5d", self.send(Op.ReadAngle))

    def get_pos(self):
        return struct.unpack("3d", self.send(Op.ReadPos))
