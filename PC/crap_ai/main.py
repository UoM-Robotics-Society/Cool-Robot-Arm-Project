import chess
import time

from . import ai
from .util import convert_to_fen, read_board_network


alphabeta_ai = ai.AI_AlphaBeta()


while True:
    #console input
    # move = input("enter move : ")
    # move = chess.Move.from_uci(move)
    # board.push(move)
    # print(board)

    #webserver input

    board = chess.Board(convert_to_fen(read_board_network()))
    #print(board)

    #ai move

    start_time = time.time()
    ai_move = alphabeta_ai.get_move(board, 3)
    print("--- %s seconds ---" % (time.time() - start_time))
    print(ai_move)
    board.push(ai_move)
    print(board)
    print("================================")

    time.sleep(4)
