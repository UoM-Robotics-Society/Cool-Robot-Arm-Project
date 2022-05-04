import chess
import time

from . import ai
from .util import convert_to_fen, read_board_network


alphabeta_ai = ai.AI_AlphaBeta()


def get_move(url, depth=3):
    board = chess.Board(convert_to_fen(read_board_network(url)))

    start_time = time.time()
    ai_move = alphabeta_ai.get_move(board, depth)
    dt = time.time() - start_time

    return ai_move, dt
