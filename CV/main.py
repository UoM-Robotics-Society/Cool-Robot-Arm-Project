import threading
import pickle
import os

import cv2
import cv2.aruco as aruco
import numpy as np
from tkinter import Frame, Tk, Label, IntVar
from tkinter.ttk import Button, Checkbutton
from flask import Flask

from util import extrapolate_points, draw_points


ROOK, BISHOP, KNIGHT, KING, QUEEN, PAWN = range(6)
BLACK, WHITE = range(2)
NAME = {
    ROOK: "Rook",
    BISHOP: "Bishop",
    KNIGHT: "Knight",
    KING: "King",
    QUEEN: "Queen",
    PAWN: "Pawn",
}

SHOW_MASK = False
SHOW_ARMASK = False
SHOW_CHESS_MARKERS = False
SHOW_EXTRAPOLATED = False
SHOW_BOARD = False

SHOW_REGIONS = True

class Main:
    PIECES_WAIT = 15
    ALLOCATIONS = {
        (0, 0): (ROOK, WHITE),
        (1, 0): (KNIGHT, WHITE),
        (2, 0): (BISHOP, WHITE),
        (3, 0): (KING, WHITE),
        (4, 0): (QUEEN, WHITE),
        (5, 0): (BISHOP, WHITE),
        (6, 0): (KNIGHT, WHITE),
        (7, 0): (ROOK, WHITE),
        (0, 1): (PAWN, WHITE),
        (1, 1): (PAWN, WHITE),
        (2, 1): (PAWN, WHITE),
        (3, 1): (PAWN, WHITE),
        (4, 1): (PAWN, WHITE),
        (5, 1): (PAWN, WHITE),
        (6, 1): (PAWN, WHITE),
        (7, 1): (PAWN, WHITE),
        (0, 6): (PAWN, BLACK),
        (1, 6): (PAWN, BLACK),
        (2, 6): (PAWN, BLACK),
        (3, 6): (PAWN, BLACK),
        (4, 6): (PAWN, BLACK),
        (5, 6): (PAWN, BLACK),
        (6, 6): (PAWN, BLACK),
        (7, 6): (PAWN, BLACK),
        (0, 7): (ROOK, BLACK),
        (1, 7): (KNIGHT, BLACK),
        (2, 7): (BISHOP, BLACK),
        (3, 7): (KING, BLACK),
        (4, 7): (QUEEN, BLACK),
        (5, 7): (BISHOP, BLACK),
        (6, 7): (KNIGHT, BLACK),
        (7, 7): (ROOK, BLACK),
    }

    def __init__(self, camera=0):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

        self.vid = cv2.VideoCapture(camera, cv2.CAP_DSHOW)
        self.vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.tk = Tk()
        Button(self.tk, text="Save Calibration", command=self.save).pack()
        Button(self.tk, text="Load Calibration", command=self.load).pack()
        Button(self.tk, text="Bake Allocations", command=self.bake_allocations).pack()

        self.live_cal = IntVar()
        self.live_cal.set(1)
        f = Frame(self.tk)
        Label(f, text="Live Calibration").pack(side="left")
        Checkbutton(f, variable=self.live_cal, state="on").pack(side="right")
        f.pack()

        self.sharpen = IntVar()
        self.sharpen.set(1)
        f = Frame(self.tk)
        Label(f, text="Sharpen Image").pack(side="left")
        Checkbutton(f, variable=self.sharpen, state="on").pack(side="right")
        f.pack()

        self.pieces = {}
        self.piece_allocations = {}

        self.square_w = 100

        self.grey = None
        self.points = None

        self.app = Flask(__name__)
        self.app.route("/")(self.flask_route)

    def bake_allocations(self):
        self.piece_allocations.clear()
        for i in self.pieces:
            allocation = self.ALLOCATIONS.get((self.pieces[i][0], self.pieces[i][1]))
            if allocation is not None:
                self.piece_allocations[i] = allocation

    def save(self):
        with open("points.pkl", "wb") as f:
            pickle.dump(self.points, f)

    def load(self):
        if os.path.exists("points.pkl"):
            with open("points.pkl", "rb") as f:
                self.points = pickle.load(f)

    def identify_aruco(self, img, do_mask=True):
        if do_mask:
            grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, masked = cv2.threshold(grey, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            if SHOW_ARMASK:
                cv2.imshow("armask", masked)
        else:
            masked = img

        corners, ids, _ = aruco.detectMarkers(
            masked, self.aruco_dict, parameters=self.aruco_params
        )
        aruco.drawDetectedMarkers(img, corners, ids)
        return corners, ids

    def get_frame(self):
        ret, img = self.vid.read()
        if not ret:
            print("TITSUP")
            quit()

        if self.sharpen.get():
            blurred = cv2.GaussianBlur(img, (0, 0), 3)
            return cv2.addWeighted(img, 1.5, blurred, -0.5, 0, blurred)

        return img

    def get_mask(self, img):
        self.grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, msk = cv2.threshold(
            self.grey, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
        )

        if SHOW_MASK:
            cv2.imshow("mask", msk)

        return msk

    def extract(self, img, points, w=50, h=50, p=0):
        out_pts = np.float32(
            [(p, p), (w - 1 - p, p), (p, h - 1 - p), (w - 1 - p, h - 1 - p)]
        )
        matrix = cv2.getPerspectiveTransform(points, out_pts)
        return cv2.warpPerspective(img, matrix, (w, h), flags=cv2.INTER_LINEAR)

    def get_8x8_regions(self, img, points):
        regions = [[None for _ in range(8)] for _ in range(8)]
        for x in range(8):
            for y in range(8):
                regions[x][y] = self.extract(
                    img,
                    np.float32(
                        [
                            points[x][y],
                            points[x + 1][y],
                            points[x][y + 1],
                            points[x + 1][y + 1],
                        ]
                    ),
                    self.square_w,
                    self.square_w,
                )
        return regions

    def stitch_8x8(self, regions):
        full_w, full_h = self.square_w * 8 + 90, self.square_w * 8 + 90
        out = np.zeros((full_w, full_h, 3), dtype=np.uint8)
        out[:] = 255
        for x in range(8):
            for y in range(8):
                out[
                    y * (self.square_w + 10) + 10 : (y + 1) * (self.square_w + 10),
                    x * (self.square_w + 10) + 10 : (x + 1) * (self.square_w + 10),
                ] = regions[x][y]
        return out

    def process_regions(self, regions):
        # for x in range(8):
        #     for y in range(8):
        #         self.identify_aruco(regions[x][y])

        if SHOW_REGIONS:
            stitched = self.stitch_8x8(regions)
            cv2.imshow("squares", stitched)

    def update_cal(self, img):
        msk = self.get_mask(img)
        # Extract chess-board
        krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
        dlt = cv2.dilate(msk, krn, iterations=5)
        res = 255 - cv2.bitwise_and(dlt, msk)

        # Displaying chess-board features
        res = np.uint8(res)
        ret, corners = cv2.findChessboardCorners(
            res,
            (7, 7),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if ret:
            dx = corners[0][0][0] - corners[6][0][0]
            dy = corners[0][0][1] - corners[6][0][1]
            if abs(dy) > abs(dx):
                corners = np.reshape(corners, (7, 7, 1, 2))
                corners = np.rot90(corners, 1, axes=(0, 1))
                corners = np.reshape(corners, (49, 1, 2))
            dy = corners[0][0][1] - corners[45][0][1]
            if dy > 0:
                corners = corners[::-1]

        objp = np.zeros((7 * 7, 3), np.float32)
        objp[:, :2] = np.mgrid[0:7, 0:7].T.reshape(-1, 2) * 30

        if ret:
            self.points = extrapolate_points((7, 7), corners)

            if SHOW_CHESS_MARKERS:
                fnl = cv2.drawChessboardCorners(img.copy(), (7, 7), corners, ret)
                cv2.imshow("markers", fnl)

    def show_focus_region(self, img):
        if self.points:

            pad = 10
            focus = self.extract(
                img,
                np.float32(
                    [
                        self.points[0][0],
                        self.points[8][0],
                        self.points[0][8],
                        self.points[8][8],
                    ]
                ),
                self.square_w * 8,
                self.square_w * 8,
                pad,
            )
            if SHOW_EXTRAPOLATED:
                cb = draw_points(img, self.points)
                cv2.imshow("extrapolated", cb)
            corners, ids = self.identify_aruco(focus, True)

            for i in {**self.pieces}:
                self.pieces[i][2] -= 1
                if self.pieces[i][2] == 0:
                    self.pieces.pop(i)

            if corners is not None and ids is not None:
                for points, id_ in zip(corners, ids):
                    centre = (np.sum(points, axis=1) / 4)[0]
                    x = int((centre[0] - pad) // self.square_w)
                    y = int((centre[1] - pad) // self.square_w)
                    self.pieces[id_[0]] = [x, y, self.PIECES_WAIT]

                    if SHOW_BOARD:
                        cv2.circle(focus, (int(centre[0]), int(centre[1])), 5, (0, 255, 0))
                        cv2.putText(
                            focus,
                            f"({x},{y})",
                            (int(centre[0]), int(centre[1]) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 255),
                            2,
                        )
            if SHOW_BOARD:
                cv2.imshow("board", focus)

    def serialize(self):
        out = ""
        for y in range(8):
            line = []

            for x in range(8):
                for piece in self.pieces:
                    x1, y1, _ = self.pieces[piece]
                    if x1 == x and y1 == y:
                        allocation = self.piece_allocations.get(piece)
                        if allocation is None:
                            line.append("??")
                        else:
                            line.append(("B" if allocation[1] else "W") + "RBNKQP"[allocation[0]])
                        break
                else:
                    line.append("--")
            out += " ".join(line) + "\n"
        return out

    def flask_route(self):
        return self.serialize()

    def detect(self, img):
        if self.live_cal.get():
            self.update_cal(img)

        if self.points:
            regions = self.get_8x8_regions(img, self.points)

            for piece in self.pieces:
                (x, y, _) = self.pieces[piece]
                cv2.putText(
                    regions[x][y],
                    str(piece),
                    (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (255, 0, 0),
                    2,
                )
                allocation = self.piece_allocations.get(piece)
                if allocation is not None:
                    cv2.putText(
                        regions[x][y],
                        "Black" if allocation[1] else "White",
                        (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 0, 255),
                        2,
                    )
                    cv2.putText(
                        regions[x][y],
                        NAME[allocation[0]],
                        (20, 85),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )

            self.show_focus_region(img)
            self.process_regions(regions)

    def mainloop(self):
        threading.Thread(target=self.app.run, daemon=True).start()

        while True:
            img = self.get_frame()
            self.detect(img)
            cv2.waitKey(1)
            self.tk.update()


if __name__ == "__main__":
    Main(4).mainloop()
