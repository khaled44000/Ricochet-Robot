
from collections import deque, defaultdict
import math, time
import json
import csv

from colorama import init
from termcolor import colored

BOARD_SIZE = 16
DEBUG = True

init()

# (X, Y) robot de localisation essaie d'arriver à L'objectif
default_goal = (0, 15)
default_robot = "red"


# dictionnaire de la couleur du robot au tuple (x, y)
Depart_Robots = {}
# x, y tuples
# nous utilisons .5 pour désigner un mur entre deux espaces pour le robot
global_walls = []

""""***************Resoluttion***************************"""


def BFS(start_state, goal_robot_name, goal, movable_robots=None, blacklist_limit=20, cost_limit=20):
    """trouver le plus petit nombre de mouvements à résoudre, compte tenu d'un certain nombre d'options.
    start_state: conditions initiales
    goal_robot_name: quel robot nous essayons de déplacer
    goal: (x,y) l'objectif
    movable_robots: liste des noms de robots que nous pouvons déplacer
    blacklist_limit: max fois nous regarderons un endroit"""

    q = deque()  # une file d'attente pour garder une trace
    q.append(start_state)

    # pour signaler
    cost = 0
    t0 = time.time()

    # nous utilisons une liste noire mondiale pour nous empêcher de visiter le même point trop de fois
    blacklist = {name: defaultdict(int) for name in start_state["robots"]}
    blacklist["limit"] = blacklist_limit

    # quels robots sommes-nous autorisés à déplacer?
    if movable_robots is None:
        movable_robots = start_state["robots"].keys()

    while len(q) > 0:
        state = q.popleft()

        if state["cost"] > cost_limit:
            return None

        if DEBUG:
            new_cost = state["cost"]
            if new_cost > cost:
                cost = new_cost
                print("Eatpe: {} Temps: {}".format(cost, time.time() - t0))

        next_states = []
        for robot_name in movable_robots:
            moves = get_robot_moves(robot_name, state)
            next_states.extend(get_next_states(robot_name, moves, state, blacklist))

        for next_state in next_states:
            if win(next_state, goal_robot_name, goal):
                return next_state
            q.append(next_state)

    # manqué d'options de recherche
    return None


def full_solve(start_state, goal_robot_name, goal):
    """Résout le tableau en appelant BFS plusieurs fois avec des paramètres différents.
    1. déplacez seulement le robot d'objectif
    2. résoudre avec tous les robots, faible profondeur de liste noire
    3. résoudre avec tous les robots, haute profondeur de liste noire"""
    result = None
    print("Essayer de déplacer seulement le robot principal")
    result = BFS(start_state, goal_robot_name, goal,
                        movable_robots=[goal_robot_name],
                        blacklist_limit=float('inf'))
    if result:
        return result

    print("déplacer tous les robots, avec une faible limite de liste noire")
    result = BFS(start_state, goal_robot_name, goal,
                        movable_robots=None,
                        blacklist_limit=20)
    if result:
        return result

    print("déplacer 1 autre robot à la fois, faible liste noire")
    other_robots = [rob for rob in start_state["robots"] if rob != goal_robot_name]
    for robot in other_robots:
        print("En essayant {}".format(robot))
        result = BFS(start_state, goal_robot_name, goal,
                            movable_robots=[goal_robot_name, robot],
                            blacklist_limit=200)
        if result:
            return result

    print("déplacer 1 autre robot à la fois")
    other_robots = [rob for rob in start_state["robots"] if rob != goal_robot_name]
    for robot in other_robots:
        print("Trying {}".format(robot))
        result = BFS(start_state, goal_robot_name, goal,
                            movable_robots=[goal_robot_name, robot],
                            blacklist_limit=2000)
        if result:
            return result

    print("déplacer tous les robots, avec une limite de blacklist élevée")
    result = BFS(start_state, goal_robot_name, goal,
                        movable_robots=None,
                        blacklist_limit=2000)
    if result:
        return result
    while result == None:
        print("test")

    return None

def BFS(start_state, goal_robot_name, goal, movable_robots=None, blacklist_limit=20):
    """trouver le plus petit nombre de mouvements à résoudre, compte tenu d'un certain nombre d'options.
    start_state: conditions initiales
    goal_robot_name:quel robot nous essayons de déplacer
    goal: (x,y) l'objectif
    movable_robots: liste des noms de robots que nous pouvons déplacer
    blacklist_limit: max fois nous regarderons une fois lieu
    cost_limit: max cout"""
    q = deque()  # une file d'attente pour garder une trace des deplacements
    q.append(start_state)

    # pour signaler
    cost = 0
    t0 = time.time()

    # nous utilisons une liste noire mondiale pour nous empêcher de visiter le même point trop de fois
    blacklist = {name: defaultdict(int) for name in start_state["robots"]}
    blacklist["limit"] = blacklist_limit

    # quels robots sommes-nous autorisés à déplacer?
    if movable_robots is None:
        movable_robots = start_state["robots"].keys()

    while len(q) > 0:
        state = q.popleft()

        if DEBUG:
            new_cost = state["cost"]
            if new_cost > cost:
                cost = new_cost
                print("Etape: {} temps: {}".format(cost, time.time() - t0))

        next_states = []
        for robot_name in movable_robots:
            moves = get_robot_moves(robot_name, state)
            next_states.extend(get_next_states(robot_name, moves, state, blacklist))

        for next_state in next_states:
            if win(next_state, goal_robot_name, goal):
                return next_state
            q.append(next_state)

    # manqué d'options de recherche
    return None


def get_robot_moves(robot_name, state, walls=global_walls):
    """renvoie tous les mouvements suivants possibles pour le robot donné.
    robot_name: nom robot
    state: etat 
    returns: tuples de coeurs que le robot peut déplacer. (up, down, right, left)"""
    current_x, current_y = state["robots"][robot_name]

    # trouver des murs et des robots avec ceux-ci x coord or y coord
    if extreme_cache is None:
        cache_wall_extremes()

    up_y, down_y, right_x, left_x = extreme_cache[(current_x, current_y)]
    xs, ys = [right_x, left_x], [up_y, down_y]  # ajouter des extrêmes de murs à la liste
    to_check = list(state["robots"].values())  # puis vérifiez les robots

    for x, y in to_check:
        if x == current_x:
            ys.append(y)
        if y == current_y:
            xs.append(x)

            # trouver les murs / robots les plus proches dans toutes les directions
    up_y, down_y = extremes(current_y, ys)
    right_x, left_x = extremes(current_x, xs)

    # créer des tuples de directions (up, down, right, left)
    return ((current_x, up_y - 1),  #besoin d'aller par 1, pour les endroits robot peut réellement aller
            (current_x, down_y + 1),
            (right_x - 1, current_y),
            (left_x + 1, current_y))


# State Vector
# dict(robots=dict(color=(x,y), cost=0, prev_state=None))

def extremes(current, arr):
    """Trouvez les spots disponibles les plus hauts et les plus bas,
     compte tenu de la valeur actuelle et de la gamme d'obstacles.
     retourne (haut, bas) les emplacements disponibles"""
    up = BOARD_SIZE
    down = -1
    for v in arr:
        if v > current and v < up:
            up = v
        elif v < current and v > down:
            down = v

    # fix rounding of walls/robots
    up = math.ceil(up)
    down = math.floor(down)
    return up, down



extreme_cache = None
def cache_wall_extremes(walls=global_walls):
    """Pour chaque emplacement sur le tableau, créez une liste des positions extrêmes des murs"""
    global extreme_cache
    extreme_cache = {}
    for current_x in range(BOARD_SIZE):
        for current_y in range(BOARD_SIZE):

            xs, ys = [], []
            for x, y in walls:
                if x == current_x:
                    ys.append(y)
                if y == current_y:
                    xs.append(x)

            # trouver les murs / robots les plus proches dans toutes les directions
            up_y, down_y = extremes(current_y, ys)
            right_x, left_x = extremes(current_x, xs)
            """print("Haut " + format(up_y) )
            print("bas " + format(down_y))
            print("droit " + format(right_x))
            print("gauche " + format(left_x))"""

            extreme_cache[(current_x, current_y)] = (up_y, down_y, right_x, left_x)
           # print(format(extreme_cache))

def get_next_states(robot_name, moves, state, blacklist):
    """packages next moves into actual states.
    robot_name: which robot we're moving.
    moves: a tuple of move coords that are possible
    state: current state vector.
    blacklist: n'envoyez pas un robot à un endroit suffisamment exploré"""
    next_states = []
    for coord in moves:  # new coordinates for the moved robot

        # ignore moves that are stuck in the same place
        if coord == state["robots"][robot_name]:
            continue
        # ignore moves that bring us back to the robot's previous location
        elif state["prev_state"] is not None and coord == state["prev_state"]["robots"][robot_name]:
            continue
        # ignore moves that bring us to a very explored location
        elif blacklist[robot_name][coord] > blacklist["limit"]:
            continue
        # let's try it!
        else:
            blacklist[robot_name][coord] += 1

            # create a new state to add to the queue
            s = {}
            s["robots"] = state["robots"].copy()
            s["robots"][robot_name] = coord
            s["cost"] = state["cost"] + 1
            s["prev_state"] = state
            next_states.append(s)
    return next_states

def win(state, robot_name, goal):
    """Vérifie si l'état actuel gagne"""
    return state["robots"][robot_name] == goal
"""************************************************************************"""

"""*************Affichage de la carte **************"""
def print_path(state, robot_name, goal=None):
    """Afficher tous les états le long du chemin que nous avons pris"""
    count = 0
    while state is not None:
        print_board(state["robots"], global_walls, goal)
        state = state["prev_state"]
        count += 1
    print("nombres des couts: {}".format(count))


def print_board(robots, walls, goal=None):
    """Dessinez le tableau avec l'art ascii d'une manière jolie.
     Il y a un 'emplacement' à chaque espacement de 0,5.
     Chaque 'emplacement' est rendu avec deux caractères.
    robots: dictionary of name: (x,y)
    walls: list of (x,y) tuples
    goal: optional (x,y)"""
    board = get_empty_board(BOARD_SIZE)

    # add walls
    for x, y in walls:
        if x != int(x):  #changer de rendu selon que x ou y 
            wall = " │"
        else:
            wall = "——"
        board[int(y * 2)][int(x * 2)] = wall

    # add goal
    if goal is not None:
        x, y = goal
        board[int(y * 2)][int(x * 2)] = render_item("goal")

    # add robots
    for name, (x, y) in robots.items():
        board[int(y * 2)][int(x * 2)] = render_item(name)

    board.reverse()  # parce que l'indexation de liste est à l'envers
    board = draw_frame(board)
    board = draw_corners(board)

    # combine into nice string and print
    for row in board:
        print(colored("".join(row)))


def draw_frame(board):
    """Draws a nice pipe boarder around a board"""
    n_rows = len(board)
    n_cols = len(board[0])
    top_row = [" ┌"] + ["——"] * n_cols + ["—┐"]
    bottom_row = [" └"] + ["——"] * n_cols + ["—┘"]

    new_board = []
    for row in board:
        new_row = [" |"] + row + [" |"]
        new_board.append(new_row)
    return [top_row] + new_board + [bottom_row]

def draw_corners(board):
    """Inserts fancy corner pipe characters"""
    for y in range(2, len(board)-2):
        for x in range(2, len(board[0])-2):
            if board[y+1][x] == " │" and board[y][x+1] == "——":
                board[y][x] = " ┌"
            elif board[y-1][x] == " │" and board[y][x+1] == "——":
                board[y][x] = " └"
            elif board[y+1][x] == " │" and board[y][x-1] == "——":
                board[y][x] = "—┐"
            elif board[y-1][x] == " │" and board[y][x-1] == "——":
                board[y][x] = "—┘"

    # handle frames as special cases
    for y in range(1, len(board)-1):
        if board[y][-2] == "——":
            board[y][-1] = "—┤"
        if board[y][1] == "——":
            board[y][0] = " ├"

    for x in range(1, len(board[2])-1):
        if board[-2][x] == " │":
            board[-1][x] = "—┴"
        if board[1][x] == " │":
            board[0][x] = "—┬"

    return board


def render_item(name):
    """Print a robot or goal name and color"""
    # color defs
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YEL = '\033[93m'
    RED = '\033[31m'
    ENDC = '\033[0m'
    CYAN = "\033[1;36m"

    if name == "goal":
        color = CYAN
    elif name == "green":
        color = GREEN
    elif name == "red":
        color = RED
    elif name == "blue":
        color = BLUE
    elif name == "yellow":
        color = YEL
    else:
        color = ""

    return color + name[0:2].upper() + ENDC


def get_empty_board(n):
    """Render empty board, to add stuff to later"""
    board = []
    for y in range(BOARD_SIZE * 2 - 1):
        board.append([])
        for x in range(BOARD_SIZE * 2 - 1):
            if x % 2 == 0 and y % 2 == 0:
                board[y].append(" ·")  # mark a spot for a robot
            else:
                board[y].append("  ")

    return board

Fichier_Depart_Robots = None
Fichier_Murs = None

def read_Depart_Robots():
    global Fichier_Depart_Robots
    if Fichier_Depart_Robots== None:
        Fichier_Depart_Robots= input("Entrer le nom du fichier de départ des robots :")
    if (Fichier_Depart_Robots is not None):
        data = defaultdict(list)
        reader = csv.DictReader('depart' + Fichier_Depart_Robots + '.csv')
        csvfile = open('depart' + Fichier_Depart_Robots + '.csv' , 'r')

        reader = csv.DictReader(open('depart' + Fichier_Depart_Robots + '.csv'))
        for row in reader:
            for column, value in row.items():
                if column not in Depart_Robots:
                    Depart_Robots[column] = ()
                Depart_Robots[column] = Depart_Robots[column] + (int(value),)
        print(format(Depart_Robots))
        return  Depart_Robots


def read_Walls():
    global Fichier_Murs
    if  Fichier_Murs == None:
        Fichier_Murs = input("Entrer le nom du fichier des murs :")  # 1 ou 2 ou 3 ou 4 ou 5
    if (Fichier_Murs is not None):
        with open('mur' + Fichier_Murs + '.csv', 'r') as csvfile:
            csvReader = csv.reader(csvfile, delimiter=';', quotechar='"')
            next(csvReader)
            for row in csvReader:
                if row:
                    X = row[0]
                    Y = row[1]
                    global_walls.append((float(X), float(Y)))
            print(format(global_walls))

def main():
    read_Depart_Robots()
    read_Walls()

    goal = default_goal
    robot_name = default_robot
    state = {"robots": Depart_Robots, "cost": 0, "prev_state": None}

    winning_state = full_solve(state, robot_name, goal)
    if winning_state is not None:
        print_path(winning_state, robot_name, goal)
        print("Gagné!")
    else:
        print_path(state, robot_name, goal)
        print("Perdu")

main()