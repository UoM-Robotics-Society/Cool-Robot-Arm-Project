import threading
import serial
import signal
import time

from tkinter import Frame, Tk, Label
from tkinter.ttk import Button, Separator

from crap_arm import CRAP, NACKException
from crap_ai import get_move
from crap_ai.util import read_board_network

from calib import CRAPCalibration, HOME


UNICODE_CHARS = {
    "BK": "♚",
    "BQ": "♛",
    "BB": "♝",
    "BN": "♞",
    "BR": "♜",
    "BP": "♟",
    "WK": "♔",
    "WQ": "♕",
    "WB": "♗",
    "WN": "♘",
    "WR": "♖",
    "WP": "♙"
}

AI_STATE = {
    "idle": dict(text="Status: Idle", fg="black"),
    "thinking": dict(text="Status: Thinking", fg="darkorange"),
    "ready": dict(text="Status: Ready", fg="green"),
    "failed": dict(text="Status: Failed", fg="red")
}
ARM_STATE = {
    "discon": dict(text="Disconnected", fg="red"),
    "con": dict(text="Status: Connected", fg="green"),
}
FONT = ("Segoe UI", 16)

DEPTH = 5


class CRAPController:
    UPSTREAM = "http://127.0.0.1:5000"

    def __init__(self):
        self.calib = CRAPCalibration(self)

        self.tk = Tk()
        self.tk.title("CRAP Controller")
        self.tk.resizable(False, False)
        self.tk.protocol("WM_DELETE_WINDOW", self._tk_delete)

        self.statusbar = Label(self.tk, text="", bd=1, relief="sunken", anchor="w")
        self.statusbar.grid(row=1, column=0, columnspan=3, sticky="WE", pady=(5, 0))

        self.crap = None
        self.message = None

        chess_frame = Frame(self.tk)
        self._make_chess(chess_frame)
        chess_frame.grid(row=0, column=0)

        ai_frame = Frame(self.tk)
        self._make_ai(ai_frame)
        ai_frame.grid(row=0, column=1, sticky="N")

        arm_frame = Frame(self.tk)
        self._make_arm(arm_frame)
        arm_frame.grid(row=0, column=2, sticky="N")

        self.thinking = False
        self.move = None
        self.move_time = None
        self.arm_busy = False

        self.corners = [None] * 4

        self.running = True

    def _tk_delete(self):
        self.running = False
        self.tk = None

    def _make_chess(self, frame):
        self.chess_tiles = {}

        for y in range(8):
            for x in range(8):
                lab = Label(frame, borderwidth="2", relief="groove", width=2, height=1, font=("Segoe UI", 32))
                lab.grid(row=x, column=y)
                self.chess_tiles[(x, y)] = lab

        self.board_stat = Label(frame, text="Board State", font=FONT)
        self.board_stat.grid(row=8, column=0, columnspan=8)
        Label(frame, text=self.UPSTREAM).grid(row=9, column=0, columnspan=8)

    def _make_ai(self, frame):
        Label(frame, text="CRAP AI", font=FONT).pack()
        self.ai_status = Label(frame, **AI_STATE["idle"], width=16, font=FONT)
        self.ai_status.pack()
        self.ai_move = Label(frame)
        self.ai_move.pack()
        self.ai_extra = Label(frame)
        self.ai_extra.pack()

        self.ai_button = Button(frame, command=self.make_ai_move, text="Get AI Move")
        self.ai_button.pack()
        self.arm_button = Button(frame, command=self.make_arm_move, text="Perform AI Move", state="disabled")
        self.arm_button.pack()
        self.clear_button = Button(frame, command=self.clear_ai_move, text="Reject AI Move", state="disabled")
        self.clear_button.pack()

    def _make_arm(self, frame):
        Label(frame, text="CRAP Arm", font=FONT).pack()

        self.arm_stat = Label(frame, font=FONT)
        self.arm_stat.pack()

        self.arm_controls = []

        self.arm_controls.append(Button(frame, command=lambda *_: self.do_arm("beep"), text="Beep"))
        self.arm_controls.append(Button(frame, command=lambda *_: self.do_arm("home"), text="Home arm"))
        self.arm_controls.append(Button(frame, command=lambda *_: self.do_arm("unlock"), text="Unlock servos"))
        self.arm_controls.append(Button(frame, command=lambda *_: self.do_arm("selfcheck"), text="Selftest"))
        self.arm_controls.append(Button(frame, command=lambda *_: self.do_arm("selfcheck_ik"), text="Selftest IK"))

        self.arm_controls.append(Button(frame, command=lambda *_: self.arm_corner(0), text="Corner 0"))
        self.arm_controls.append(Button(frame, command=lambda *_: self.arm_corner(1), text="Corner 1"))
        self.arm_controls.append(Button(frame, command=lambda *_: self.arm_corner(2), text="Corner 2"))
        self.arm_controls.append(Button(frame, command=lambda *_: self.arm_corner(3), text="Corner 3"))
        self.arm_controls.append(Button(frame, command=self.touchoff, text="Touchoff"))

        for i in self.arm_controls:
            i.pack()

        Separator(frame, orient="horizontal").pack(fill="x", pady=8)

        self.corner_labels = [
            Label(frame),
            Label(frame),
            Label(frame),
            Label(frame),
        ]
        for i in self.corner_labels:
            i.pack()

        Separator(frame, orient="horizontal").pack(fill="x", pady=8)
        Button(frame, command=self.calib.show, text="Calibration").pack()

    def do_arm(self, action, args=()):
        def do():
            self.arm_busy = True
            try:
                getattr(self.crap, action)(*args)
            except serial.serialutil.SerialException:
                self.crap = None
            self.arm_busy = False
        threading.Thread(target=do).start()

    def arm_corner(self, corner):
        if self.crap is None:
            return
        try:
            self.corners[corner] = self.crap.get_pos()
        except serial.serialutil.SerialException:
            self.crap = None

    def _make_ai_move(self):
        self.thinking = True
        self.move, self.move_time = get_move(self.UPSTREAM, depth=DEPTH)
        if self.move is None:
            self.move = -1
        self.thinking = False

    def make_ai_move(self):
        self.ai_button.configure(state="disabled")
        threading.Thread(target=self._make_ai_move).start()

    def touchoff(self):
        def do():
            self.arm_busy = True
            for i in self.corners:
                if i is not None:
                    try:
                        self.crap.move(i[0], i[1], i[2])
                    except NACKException:
                        self.crap.beep()
                    except serial.serialutil.SerialException:
                        self.crap = None
                        break
            self.arm_busy = False
        if self.crap is not None:
            threading.Thread(target=do).start()

    def clear_ai_move(self):
        self.move = None

    def _make_arm_move(self):
        air = self.calib.get_air()
        if air is None:
            air = HOME

        from_ = divmod(self.move.from_square, 8)
        to = divmod(self.move.to_square, 8)

        p1 = self.calib.get_angle(*from_)
        p2 = self.calib.get_angle(*to)

        self.arm_busy = True

        if p1 == HOME:
            self.crap.beep()
        else:
            self.crap.move_a(*p1)
        self.crap.move_a(*air)
        if p2 == HOME:
            self.crap.beep()
        else:
            self.crap.move_a(*p2)
        self.crap.home()

        self.arm_busy = False
        self.move = None

    def make_arm_move(self):
        threading.Thread(target=self._make_arm_move).start()

    def got_message(self, message):
        self.message = time.strftime("[%Y/%m/%d %H:%M:%S] ") + message.strip()

    def main(self):
        signal.signal(signal.SIGINT, lambda *_: setattr(self, "running", False))

        while self.running:
            if self.message:
                self.statusbar.configure(text=self.message)

            try:
                board = read_board_network(self.UPSTREAM)
            except Exception:
                board = None

            if self.crap is None:
                try:
                    self.crap = CRAP(msg_handler=self.got_message)
                except serial.serialutil.SerialException:
                    pass

            if self.crap is not None and not self.arm_busy:
                try:
                    self.crap.ping()
                except serial.serialutil.SerialException:
                    self.crap = None

            if self.crap is None:
                self.arm_stat.configure(**ARM_STATE["discon"])
                for i in self.arm_controls:
                    i.configure(state="disabled")
            else:
                self.arm_stat.configure(**ARM_STATE["con"])
                for i in self.arm_controls:
                    i.configure(state="disabled" if self.arm_busy else "normal")

            if board is not None:
                self.board_stat.configure(text="Board State (Live)", fg="darkgreen")
                for y, row in enumerate(board):
                    for x, cell in enumerate(row):
                        self.chess_tiles[(x, y)].configure(text=UNICODE_CHARS.get(cell, cell))
            else:
                self.board_stat.configure(text="Board State (Stale)", fg="red")

            for i, j in zip(self.corners, self.corner_labels):
                if i is None:
                    j.configure(text="--")
                else:
                    j.configure(text=f"{i[0]*100:.2f}cm, {i[1]*100:.2f}cm, {i[2]*100:.2f}cm")

            if self.thinking:
                self.ai_button.configure(state="disabled")
                self.arm_button.configure(state="disabled")
                self.clear_button.configure(state="disabled")
                self.ai_status.configure(**AI_STATE["thinking"])
                self.ai_move.configure(text="")
                self.ai_extra.configure(text="")
            elif self.move is None or self.move == -1:
                self.ai_button.configure(state="normal")
                self.arm_button.configure(state="disabled")
                self.clear_button.configure(state="disabled")
                self.ai_status.configure(**AI_STATE["idle" if self.move is None else "failed"])
                self.ai_move.configure(text="")
                self.ai_extra.configure(text="")
            else:
                self.ai_button.configure(state="disabled")
                self.arm_button.configure(state="disabled" if (self.arm_busy or self.crap is None) else "normal")
                self.clear_button.configure(state="normal")
                self.ai_status.configure(**AI_STATE["ready"])
                move = f"Move {self.move.from_square} to {self.move.to_square} ({self.move.uci()})"
                self.ai_move.configure(text=move)
                move_time = f"Calculted in {self.move_time * 1000:.1f}ms"
                self.ai_extra.configure(text=move_time)

            self.tk.update()
            self.calib.update()

        print("Goodbye!")
        if self.tk is not None:
            self.tk.destroy()
        if self.calib is not None:
            self.calib.tk.destroy()


if __name__ == "__main__":
    CRAPController().main()
