import pickle
import os

from tkinter import Frame, Tk, Button as TkButton
from tkinter.ttk import Button


HOME = (0, 0, 0, 0, 0)


def bezier(p0, p1, p2, t):
    return (p0 * t * t) + (p1 * 2 * t * (1 - t)) + (p2 * (1 - t) * (1 - t))


def bezier_c(p0, pc, p2, t):
    p1 = (2 * pc) - (p0 / 2) - (p2 / 2)
    return bezier(p0, p1, p2, t)


def bezier_angles(p0, pc, p2, t):
    return (
        bezier_c(p0[0], pc[0], p2[0], t),
        bezier_c(p0[1], pc[1], p2[1], t),
        bezier_c(p0[2], pc[2], p2[2], t),
        bezier_c(p0[3], pc[3], p2[3], t),
        bezier_c(p0[4], pc[4], p2[4], t),
    )


class CRAPCalibration:
    CONFIG = "whey.pickle"

    def __init__(self, controller):
        self.controller = controller

        self.positions = {}
        self.load()

        self.tk = Tk()
        self.tk.title("CRAP Calibrator")
        self.tk.resizable(False, False)

        self.tk.protocol("WM_DELETE_WINDOW", self.hide)
        self.hide()

        self.buttons = {}
        self.active = None

        def click(x, y):
            def command(event):
                lock = event.num != 1

                btn = self.buttons[(x, y)]

                if self.active != (x, y):
                    if self.active:
                        self.buttons[self.active].configure(
                            bg="green" if self.active in self.positions else "SystemButtonFace"
                        )

                    pos = self._get_angle(x, y)
                    if lock:
                        if pos is None:
                            self.controller.crap.beep()
                        else:
                            self.controller.crap.move_a(*pos, 250)
                        self.active = None
                        return

                    self.active = (x, y)
                    if pos is not None:
                        self.controller.crap.move_a(*pos, 250)
                        btn.configure(bg="orange")
                    else:
                        btn.configure(bg="red")

                    if not lock:
                        self.controller.crap.unlock()
                    self.controller.crap.beep()
                    self.cancel_b.configure(state="enabled")
                else:
                    self.active = None
                    self.positions[(x, y)] = self.controller.crap.get_angles()
                    self.controller.crap.move_a(*self.positions[(x, y)], 0)
                    self.save()
                    self.controller.crap.beep()
                    btn.configure(bg="green")
                    self.cancel_b.configure(state="disabled")
            return command

        for y in range(10):
            gy = y
            if y == 4:
                y = "whey"
            elif y > 4:
                y -= 1

            if y == 8:
                y = "air"

            for x in range(9 if y != "air" else 1):
                gx = x
                if x == 4:
                    x = "whey"
                elif x > 4:
                    x -= 1

                btn = TkButton(self.tk, width=2, relief="groove")
                btn.bind("<Button>", click(x, y))
                btn.configure(bg="green" if (x, y) in self.positions else "SystemButtonFace")
                self.buttons[(x, y)] = btn

                if x == "whey" or y == "whey":
                    btn.configure(text="W")
                if y == "air":
                    btn.configure(text="A")

                btn.grid(row=gy, column=gx)

        status = Frame(self.tk)
        status.grid(row=10, column=0, columnspan=9, sticky="WE")

        self.cancel_b = Button(status, text="Cancel", command=self.cancel)
        self.cancel_b.pack(side="left")
        self.cancel_b.configure(state="disabled")
        Button(status, text="Exit", command=self.hide).pack(side="right")

    def save(self):
        with open(self.CONFIG, "wb") as f:
            pickle.dump(self.positions, f)

    def load(self):
        if os.path.exists(self.CONFIG):
            with open(self.CONFIG, "rb") as f:
                self.positions = pickle.load(f)

    def cancel(self):
        if self.active:
            self.buttons[self.active].configure(
                bg="green" if self.active in self.positions else "SystemButtonFace"
            )
            self.active = None
            self.controller.crap.home()
        self.cancel_b.configure(state="disabled")

    def get_air(self):
        return self.positions.get((0, "air"))

    def _get_angle(self, x, y):
        if y == "air":
            return self.get_air()

        if (x, y) in self.positions:
            return self.positions[(x, y)]

        if (x, 0) in self.positions and (x, 7) in self.positions and (x, "whey") in self.positions:
            p0 = self.positions[(x, 7)]
            pc = self.positions[(x, "whey")]
            p2 = self.positions[(x, 0)]

            return bezier_angles(p0, pc, p2, y / 7)

        if x in (0, 7, "whey"):
            return None

        p2 = self._get_angle(0, y)
        pc = self._get_angle("whey", y)
        p0 = self._get_angle(7, y)

        if None in (p0, pc, p2):
            return None

        return bezier_angles(p0, pc, p2, x / 7)

    def get_angle(self, x, y):
        if (angle := self._get_angle(x, y)) is not None:
            return angle
        return HOME

    def show(self):
        self.tk.deiconify()
        self.controller.arm_busy = True

    def hide(self):
        self.tk.withdraw()
        self.controller.arm_busy = False

    def update(self):
        self.tk.update()
