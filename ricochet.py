
from collections import deque, defaultdict
import math, time
import json
import csv

from colorama import init
from termcolor import colored

TAILLE_DE_LA_CARTE = 16

init()

# (X, Y) robot de localisation essaie d'arriver à L'objectif
objectif = (15  , 0)
robot_principal = "red"


# dictionnaire de la couleur du robot au tuple (x, y)
Depart_Robots = {}
# x, y tuples
# nous utilisons .5 pour désigner un mur entre deux espaces pour le robot
murs_Globaux = []

""""***************  Résoluttion ***************************"""


def BFS(etat_de_départ, robot_principal, objectif, robots_qui_deplacent=None, limite_List_Noir=1, nbre_Coups=1):
    """trouver le plus petit nombre de mouvements à résoudre, compte tenu d'un certain nombre d'options.
    etat_de_départ: conditions initiales
    robot_principal: quel robot nous essayons de déplacer
    objectif: (x,y) l'objectif
    robots_qui_deplacent: liste des noms de robots que nous pouvons déplacer
    limite_List_Noir: nombre max de fois nous regarderons un endroit
    nbre_Coups = nbre des coups"""

    q = deque()  # une file d'attente pour garder une trace (BFS)
    q.append(etat_de_départ)

    # pour signaler
    cost = 0
    t0 = time.time()

    # j'utilises une liste noire  pour  empêcher de visiter le même point trop de fois
    blacklist = {name: defaultdict(int) for name in etat_de_départ["robots"]}
    blacklist["limit"] = limite_List_Noir

    # quels robots sommes-nous autorisés à déplacer?
    if robots_qui_deplacent is None:
        robots_qui_deplacent = etat_de_départ["robots"].keys()
    # BFS
    while len(q) > 0:
        etat = q.popleft()
        if etat["cost"] > nbre_Coups:
            return None
        if True:
            new_cost = etat["cost"]
            if new_cost > cost:
                cost = new_cost
                print("Eatpe: {} Temps: {}".format(cost, time.time() - t0))
        
        etats_suivants = []
        for nom_Robot in robots_qui_deplacent:
            deplacement = obtenir_deplacement_robots(nom_Robot, etat)
            etats_suivants.extend(obtenir_prochains_etats(nom_Robot, deplacement, etat, blacklist))
          
        for etats_suivant in etats_suivants:
            if gagner(etats_suivant, robot_principal, objectif):
                return etats_suivant
            q.append(etats_suivant)
    return None

def gagner(etat, nom_Robot, objectif):
    """Vérifie si l'état actuel gagne"""
    return etat["robots"][nom_Robot] == objectif

def resolution(etat_de_départ, robot_principal, objectif):
    """Résout le tableau en appelant BFS plusieurs fois avec des paramètres différents.
    1. déplacez seulement le robot d'objectif
    2. résoudre avec tous les robots, faible profondeur de liste noire
    3. résoudre avec tous les robots, haute profondeur de liste noire"""
    isSolution = False
    limit =  1
    coup = 1
    resultat = None
    while isSolution is False:
        limit +=1
        coup +=1
        print("Déplacement du robot principal")
        resultat = BFS(etat_de_départ, robot_principal, objectif, robots_qui_deplacent=[robot_principal], limite_List_Noir = limit, nbre_Coups=coup)
        print(format(resultat)) 
        affichage(resultat, robot_principal, objectif)
        print("solution " + format(etat_de_départ["robots"][robot_principal]))
        try:
            if resultat["robots"][robot_principal] is not None:
                if resultat["robots"][robot_principal] == objectif:
                    isSolution = True
        except:
            isSolution = False
            print("pas de solution pour ce cas ")

        print("déplacement de tous les robots, avec une faible limite de liste noire")
        resultat = BFS(etat_de_départ, robot_principal, objectif, robots_qui_deplacent=None,limite_List_Noir= limit , nbre_Coups=coup)
        print(format(resultat)) 
        affichage(resultat, robot_principal, objectif)
        try:
            if resultat["robots"][robot_principal] is not None:
                if resultat["robots"][robot_principal] == objectif:
                    isSolution = True
        except:
            isSolution = False
            print("pas de solution pour ce cas ")

        print("déplaceement d'1 autre robot à la fois, faible liste noire")
        other_robots = [rob for rob in etat_de_départ["robots"] if rob != robot_principal]
        for robot in other_robots:
            print("Trying {}".format(robot))
            resultat = BFS(etat_de_départ, robot_principal, objectif, robots_qui_deplacent=[robot_principal, robot], limite_List_Noir= limit , nbre_Coups=coup)
            print(format(resultat))
            affichage(resultat, robot_principal, objectif)
            try:
                if resultat["robots"][robot_principal] is not None:
                    if resultat["robots"][robot_principal] == objectif:
                        isSolution = True
            except:
                isSolution = False
                print("pas de solution pour ce cas ")

        print("déplacement d'1 autre robot à la fois")
        other_robots = [rob for rob in etat_de_départ["robots"] if rob != robot_principal]
        for robot in other_robots:
            print("Trying {}".format(robot))
            resultat = BFS(etat_de_départ, robot_principal, objectif, robots_qui_deplacent=[robot_principal, robot],limite_List_Noir= limit , nbre_Coups=coup)
            print(format(resultat))
            affichage(resultat, robot_principal, objectif)
            try:
                if resultat["robots"][robot_principal] is not None:
                    if resultat["robots"][robot_principal] == objectif:
                        isSolution = True
            except:
                isSolution = False
                print("pas de solution pour ce cas ")
                  
        print("déplacement de tous les robots, avec une limite de blacklist élevée")
        resultat = BFS(etat_de_départ, robot_principal, objectif, robots_qui_deplacent=None, limite_List_Noir= limit , nbre_Coups=coup)
        print(format(resultat)) 
        affichage(resultat, robot_principal, objectif)
        try:
            if resultat["robots"][robot_principal] is not None:
                if resultat["robots"][robot_principal] == objectif:
                    isSolution = True
        except:
            isSolution = False
            print("pas de solution pour ce cas ")

    return resultat

def obtenir_deplacement_robots(nom_Robot, etat, murs=murs_Globaux):
    """renvoie tous les mouvements suivants possibles pour le robot donné.
    nom_Robot: nom robot
    etat: etat 
    returns: tuples de coeurs que le robot peut déplacer. (up, down, right, left)"""
    actuel_x, actuel_y = etat["robots"][nom_Robot]

    # trouver des murs et des robots avec ceux-ci x coord or y coord
    if extreme_cache is None:
        cache_Murs_extremes()

    up_y, down_y, right_x, left_x = extreme_cache[(actuel_x, actuel_y)]
    xs, ys = [right_x, left_x], [up_y, down_y]  # ajouter des extrêmes de murs à la liste
    to_check = list(etat["robots"].values())  # puis vérifiez les robots

    for x, y in to_check:
        if x == actuel_x:
            ys.append(y)
        if y == actuel_y:
            xs.append(x)

    # trouver les murs / robots les plus proches dans toutes les directions
    up_y, down_y = extremes(actuel_y, ys)
    right_x, left_x = extremes(actuel_x, xs)

    # créer des tuples de directions (up, down, right, left)
    return ((actuel_x, up_y - 1),  #besoin d'aller par 1, pour les endroits robot peut réellement aller
            (actuel_x, down_y + 1),
            (right_x - 1, actuel_y),
            (left_x + 1, actuel_y))


# etat Vector
# dict(robots=dict(color=(x,y), cost=0, prev_state=None))

def extremes(actuel, arr):
    """Trouvez les spots disponibles les plus hauts et les plus bas,
     compte tenu de la valeur actuelle et de la gamme d'obstacles.
     retourne (haut, bas) les emplacements disponibles"""
    up = TAILLE_DE_LA_CARTE
    down = -1
    for v in arr:
        if v > actuel and v < up:
            up = v
        elif v < actuel and v > down:
            down = v

    # réparer l'arrondi des murs / robots
    up = math.ceil(up)
    down = math.floor(down)
    return up, down



extreme_cache = None
def cache_Murs_extremes(murs=murs_Globaux):
    """Pour chaque emplacement sur le tableau, créez une liste des positions extrêmes des murs"""
    global extreme_cache
    extreme_cache = {}
    for actuel_x in range(TAILLE_DE_LA_CARTE):
        for actuel_y in range(TAILLE_DE_LA_CARTE):

            xs, ys = [], []
            for x, y in murs:
                if x == actuel_x:
                    ys.append(y)
                if y == actuel_y:
                    xs.append(x)

            # trouver les murs / robots les plus proches dans toutes les directions
            up_y, down_y = extremes(actuel_y, ys)
            right_x, left_x = extremes(actuel_x, xs)
            print("Haut " + format(up_y) )
            print("bas " + format(down_y))
            print("droit " + format(right_x))
            print("gauche " + format(left_x))

            extreme_cache[(actuel_x, actuel_y)] = (up_y, down_y, right_x, left_x)
           # print(format(extreme_cache))

def obtenir_prochains_etats(nom_Robot, deplacement, etat, liste_Noir):
    """Les deplacement suivante des robots
    nom_Robot: quel robot va bouger
    deplacement: un tuple de coordonnées de mouvement possibles
    etat: vecteur d'état actuel.
    liste_Noir: n'envoyez pas un robot à un endroit déja samment exploré"""
    etats_suivants = []
    for coord in deplacement:  # nouvelles coordonnées pour le robot déplacé

        # ignorer les mouvements qui sont coincés au même endroit
        if coord == etat["robots"][nom_Robot]:
            continue
        # ignorer les mouvements qui nous ramènent à l'emplacement précédent du robot
        elif etat["prev_state"] is not None and coord == etat["prev_state"]["robots"][nom_Robot]:
            continue
        # ignorer les mouvements qui nous amènent à un endroit deja exploré
        elif liste_Noir[nom_Robot][coord] > liste_Noir["limit"]:
            continue
        else:
            liste_Noir[nom_Robot][coord] += 1

            # créer un nouvel état à ajouter à la file d'attente
            s = {}
            s["robots"] = etat["robots"].copy()
            s["robots"][nom_Robot] = coord
            s["cost"] = etat["cost"] + 1
            s["prev_state"] = etat
            etats_suivants.append(s)
    return etats_suivants


"""************************************************************************"""

"""*************Affichage de la carte **************"""
def affichage(etat, nom_Robot, objectif=None):
    """Afficher tous les états le long du chemin que nous avons pris"""
    count = 0
    while etat is not None:
        affichage_de_Carte(etat["robots"], murs_Globaux, objectif)
        etat = etat["prev_state"]
        count += 1
    print("nombres des couts: {}".format(count))


def affichage_de_Carte(robots, murs, objectif=None):
    """Dessinez le tableau avec l'art ascii.
     Il y a un 'emplacement' à chaque espacement de 0,5.
     Chaque 'emplacement' est rendu avec deux caractères.
    robots: dictionary of name: (x,y)
    murs: liste de type (x,y) tuples
    objectif: objectif (x,y)"""
    board = obtenir_emplacement_vide(TAILLE_DE_LA_CARTE)

    # ajouter  murs
    for x, y in murs:
        if x != int(x):  #changer de rendu selon que x ou y 
            wall = " │"
        else:
            wall = "——"
        board[int(y * 2)][int(x * 2)] = wall

    # ajouter l'objectif
    if objectif is not None:
        x, y = objectif
        board[int(y * 2)][int(x * 2)] = obtenir_Couleur("goal")

    # ajouter les 4 robots
    for name, (x, y) in robots.items():
        board[int(y * 2)][int(x * 2)] = obtenir_Couleur(name)

    board.reverse()  # parce que l'indexation de liste est à l'envers
    board = cadre_Carte(board)
    board = arrondir_Angles(board)

    # affichage de la carte
    for row in board:
        print(colored("".join(row)))


def cadre_Carte(board):
    """Dessine la bordure """
    n_rows = len(board)
    n_cols = len(board[0])
    top_row = [" ┌"] + ["——"] * n_cols + ["—┐"]
    bottom_row = [" └"] + ["——"] * n_cols + ["—┘"]

    new_board = []
    for row in board:
        new_row = [" |"] + row + [" |"]
        new_board.append(new_row)
    return [top_row] + new_board + [bottom_row]

def arrondir_Angles(board):
    """Dessine les coins"""
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

    # Dessine les cadres
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


def obtenir_Couleur(name):
    """Donner un couleur a chaque Robot"""
    # couleur 
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YEL = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    CYAN  = "\033[1;36m"

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


def obtenir_emplacement_vide(n):
    """Ajouter les points et l'emplacement vide"""
    bordure = []
    for y in range(TAILLE_DE_LA_CARTE * 2 - 1):
        bordure.append([])
        for x in range(TAILLE_DE_LA_CARTE * 2 - 1):
            if x % 2 == 0 and y % 2 == 0:
                bordure[y].append(" ·")  # marquer une place pour un robot
            else:
                bordure[y].append("  ")

    return bordure

Fichier_Depart_Robots = None
Fichier_Murs = None

def obtenir_Depart_Robots():
    global Fichier_Depart_Robots
    if Fichier_Depart_Robots== None:
        Fichier_Depart_Robots= input("Entrer le nom du fichier CSV qui répresnte l'emplacement des robots :")  # 1 ou 2 ou 3 ..
    if (Fichier_Depart_Robots is not None):
        reader = csv.DictReader(open('depart' + Fichier_Depart_Robots + '.csv'))
        for row in reader:
            for column, value in row.items():
                if column not in Depart_Robots:
                    Depart_Robots[column] = ()
                Depart_Robots[column] = Depart_Robots[column] + (int(value),)
        print(format(Depart_Robots))
        print("Robot principal : " + format(robot_principal))
        print("objectif: " + format(objectif))
        return  Depart_Robots


def obtenir_Carte():
    global Fichier_Murs
    if  Fichier_Murs == None:
        Fichier_Murs = input("Entrer le nom du fichier CSV pour générer la carte :")  # 1 ou 2 ou 3 ..
    if (Fichier_Murs is not None):
        with open('carte' + Fichier_Murs + '.csv', 'r') as csvfile:
            csvReader = csv.reader(csvfile, delimiter=';', quotechar='"')
            next(csvReader)
            for row in csvReader:
                if row:
                    X = row[0]
                    Y = row[1]
                    murs_Globaux.append((float(X), float(Y)))
            print(format(murs_Globaux))

def main():
    obtenir_Depart_Robots()
    obtenir_Carte()
    
    etat = {"robots": Depart_Robots, "cost": 0, "prev_state": None}
    solution =  resolution(etat, robot_principal, objectif)
    if solution is not None:
        print("")
        print("======================================== Solution optimale ============================================")
        affichage(solution, robot_principal, objectif)
        print(format(solution))
    else:
        print("======================================== Pas  solution, essayer de changer l'emplacement des robots dans le fichier d'entrer CSV ============================================")
        affichage(etat, robot_principal, objectif)

main()