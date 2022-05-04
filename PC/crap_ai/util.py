import requests


DEFAULT_WEBSERVER = "http://127.0.0.1:5000"


def convert_to_fen(board):
    peice_conversion = {
        "BK": "k",
        "BQ": "q",
        "BB": "b",
        "BN": "n",
        "BR": "r",
        "BP": "p",
        "WK": "K",
        "WQ": "Q",
        "WB": "B",
        "WN": "N",
        "WR": "R",
        "WP": "P"
    }

    fen = ""
    for x in range(8):
        count = 0
        for y in range(8):
            if board[x][y] in ("..", "--", "??"):
                count += 1
            elif count > 0:
                fen += str(count)
                fen += peice_conversion.get(board[x][y])
                count = 0
            else:
                fen += peice_conversion.get(board[x][y])
        if count > 0:
            fen += str(count)
        if x != 7:
            fen += "/"

    fen += " b KQkq - 0 1"
    return fen


def read_board_network(upstream=DEFAULT_WEBSERVER):
    resp = requests.get(upstream, timeout=0.25)
    text = resp.content.decode("latin-1").strip().split("\n")
    board = [i.split() for i in text]
    return board


def read_board(filename):
    board_matrix = []
    with open(filename, "r") as f:
        for line in f:
            line_matrix = []
            for word in line.split():
                line_matrix.append(word)
            board_matrix.append(line_matrix)

    return board_matrix


def merege_sort(moves, scores):
    return moves


def bubble_sort(moves, scores):
    for i in range(1, len(scores)):
        for j in range(0, len(scores)-1):
            if (scores[j+1] > scores[j]):
                scores[j+1], scores[j] = scores[j], scores[j+1]
                moves[j+1], moves[j] = moves[j], moves[j+1]
    return moves
