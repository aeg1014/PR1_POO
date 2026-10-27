"""
Configuración centralizada de la planta industrial.
"""

from pathlib import Path

import numpy as np


# =============================================================================
# RUTAS
# =============================================================================
CARPETA_BASE = Path(__file__).resolve().parent
CARPETA_CAPTURAS = CARPETA_BASE.parent / "capturas"

MENAGERIE_REPO = "https://github.com/google-deepmind/mujoco_menagerie.git"
MUJOCO_MENAGERIE_PATH = 'C:\\Users\\MSI Raider A18\\PycharmProjects\\PR1_POO\\PR1\\mujoco_menagerie'
# Carpeta donde se descarga Menagerie si no se encuentra en ningún sitio
CARPETA_VENDOR = CARPETA_BASE / "vendor"
# Rutas donde se busca el modelo del Panda (además de la variable de entorno
# MUJOCO_MENAGERIE_PATH, que tiene prioridad)
RUTAS_MENAGERIE = [
    CARPETA_BASE / "vendor" / "mujoco_menagerie",
    CARPETA_BASE.parent / "vendor" / "mujoco_menagerie",
    CARPETA_BASE / "mujoco_menagerie",
    CARPETA_BASE.parent / "mujoco_menagerie",
    Path.home() / "mujoco_menagerie",
]


# =============================================================================
# SIMULACIÓN
# =============================================================================
TIMESTEP = 0.002
GRAVEDAD = "0 0 -9.81"
INTEGRADOR = "RK4"


# =============================================================================
# GEOMETRÍA DE LAS CINTAS (escena.py)
# =============================================================================
BELT_HALF_LENGTH = 1.45
BELT_HALF_WIDTH = 0.28
BELT_Z = 0.12
BELT_TOP_Z = 0.18
LANE = 1.45
TURN_START = 1.08
CORNER_SWITCH = 1.27
RAIL_END_GAP = 0.06
RAIL_HALF_LENGTH = LANE - BELT_HALF_WIDTH - RAIL_END_GAP
RAIL_THICKNESS = 0.035
RAIL_HALF_HEIGHT = 0.10
RAIL_CENTER_Z = 0.22

# Separación entre la pieza y la banda al colocarla (las medidas de las
# piezas están en la sección PIEZAS, más abajo)
OBJECT_CLEARANCE_Z = 0.01

# Cámaras de inspección (posición en la escena)
CAM_HEIGHT = 0.92
CAM_FOVY = 32.0
CAM_LENS_SIDE_OFFSET = 0.10
CAM_POLE_SIDE_OFFSET = BELT_HALF_WIDTH + 0.45
CAM_OVEN_HEIGHT = 0.78
CAM_OVEN_SIDE_OFFSET = 0.46
CAM_OVEN_FOVY = 34.0
CAM_LEFT_INSPECTION_Y = 0.82
CAM_LEFT_HEAD_CLEARANCE = 0.20
CAM_LEFT_POLE_CLEARANCE = 0.52

# Cinta 5 (rama hacia el horno)
BRANCH_SPEED = 0.48
BRANCH_Y = 0.0
BRANCH_HALF_LENGTH = 1.15
BRANCH_HALF_WIDTH = 0.32
BRANCH_CENTER_X = -(LANE + BRANCH_HALF_LENGTH - 0.05)
BRANCH_OPEN_HALF = 0.38
BRANCH_RAIL_HALF = (RAIL_HALF_LENGTH - BRANCH_OPEN_HALF) / 2.0
BRANCH_RAIL_CENTER_Y = (RAIL_HALF_LENGTH + BRANCH_OPEN_HALF) / 2.0
BRANCH_RAIL_START_X = -2.00
BRANCH_RAIL_END_X = -3.48
BRANCH_RAIL_CENTER_X = 0.5 * (BRANCH_RAIL_START_X + BRANCH_RAIL_END_X)
BRANCH_RAIL_HALF_LENGTH = 0.5 * abs(BRANCH_RAIL_END_X - BRANCH_RAIL_START_X)

# Hueco en la barandilla de la Cinta 4 para que llegue el Panda
PICK_RAIL_OPEN_Y_MIN = -0.10
PICK_RAIL_OPEN_Y_MAX = 0.72
LEFT_RAIL_UPPER_CENTER_Y = 0.5 * (RAIL_HALF_LENGTH + PICK_RAIL_OPEN_Y_MAX)
LEFT_RAIL_UPPER_HALF = 0.5 * (RAIL_HALF_LENGTH - PICK_RAIL_OPEN_Y_MAX)
LEFT_RAIL_LOWER_CENTER_Y = 0.5 * (-RAIL_HALF_LENGTH + PICK_RAIL_OPEN_Y_MIN)
LEFT_RAIL_LOWER_HALF = 0.5 * (PICK_RAIL_OPEN_Y_MIN + RAIL_HALF_LENGTH)

# Horno (posición en la escena)
OVEN_CENTER_X = -4.05
OVEN_ENTRY_X = -3.58


# =============================================================================
# ROBOT PANDA (robot.py)
# =============================================================================
PANDA_PREFIX = "panda_"
PANDA_HOME_Q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=float)
PANDA_OPEN = 255.0
PANDA_CLOSED = 12.0
PANDA_FINGER_OPEN_QPOS = 0.04

ROBOT_IK_DAMPING = 2.0e-3
ROBOT_MAX_QVEL = 1.15
ROBOT_POS_GAIN = 3.0
ROBOT_ORI_GAIN = 1.4
ROBOT_NULL_GAIN = 0.35
ROBOT_TARGET_TOL = 0.035
ROBOT_ORI_TOL_RAD = np.deg2rad(4.0)
ROBOT_STATE_DWELL = 0.28
ROBOT_STATE_TIMEOUT = 7.0       # s máximos en una fase antes de reiniciar el ciclo
ROBOT_IK_LOG_PERIODO = 0.5      # s entre mensajes [IK] (2 Hz)

# Límites específicos del modo IK cartesiano manual.
ROBOT_MANUAL_POS_GAIN = 3.2
ROBOT_MANUAL_ORI_GAIN = 1.8
ROBOT_MANUAL_MAX_QVEL = 1.25

# Fuerza que retiene la pieza en la estación de recogida
ROBOT_HOLD_KP = 18.0
ROBOT_HOLD_KD = 5.0

# Site del TCP que se añade a la mano del Panda
TCP_POS = [0.0, 0.0, 0.105]

# Configuración geométrica del robot y del pick-and-place
CONFIG_ROBOT = {
    "base_x": -1.02,
    "base_y": 0.20,
    "base_z": 0.16,
    "base_yaw_deg": -90.0,
    "pick_object_ids": {0},
    "pick_trigger_y": 0.50,
    "pick_y": 0.38,
    "pick_x": -float(LANE),
    "pick_approach_z": 0.58,
    "pick_z": 0.39,
    "place_x": -1.70,
    "place_y": float(BRANCH_Y),
    "place_approach_z": 0.58,
    "place_z": 0.39,
    "branch_speed": float(BRANCH_SPEED),
}

# Isla robotizada (depende de la base del robot)
ISLAND_X = CONFIG_ROBOT["base_x"]
ISLAND_Y = CONFIG_ROBOT["base_y"]
ISLAND_Z = CONFIG_ROBOT["base_z"]
ISLAND_HALF_X = 0.18
ISLAND_HALF_Y = 0.38


# =============================================================================
# ROBOT PANDA - MODO PRUEBA IK (robot_panda.py)
# =============================================================================
MODO_PRUEBA_IK = True
IK_OBJETIVO_POSICION = [-1.45, 0.05, 0.62]
IK_OBJETIVO_RPY = [0.0, 0.0, 20.0]
IK_MARCO_ORIENTACION = "home"


# =============================================================================
# CINTAS TRANSPORTADORAS (cinta_transportadora.py)
# =============================================================================
CINTA_KP_VELOCIDAD = 7.0
CINTA_KP_CENTRADO = 12.0
CINTA_FUERZA_MAXIMA = 12.0
CINTA_INICIO_GIRO = TURN_START
CINTA_CAMBIO_CINTA = CORNER_SWITCH
CINTA_VELOCIDAD_RODILLOS = 5.5

VELOCIDAD_CINTAS = 0.55          # velocidad de las cintas 1-4
INDICE_CINTA_HORNO = 4           # posición de la Cinta 5 en la lista de cintas

# Definición de cada cinta: número, nombre, dirección, centro, giro en la
# esquina, actuadores de los rodillos y velocidad de los rodillos.
# La velocidad de transporte None significa VELOCIDAD_CINTAS.
CINTAS = [
    dict(numero=1, nombre="CINTA 1 - INFERIOR", velocidad=None,
         direccion=(1, 0), centro=(0, -LANE), giro=(0, 1),
         rodillos=("belt_act_b1", "belt_act_b2"), velocidad_rodillos=5.5),
    dict(numero=2, nombre="CINTA 2 - DERECHA", velocidad=None,
         direccion=(0, 1), centro=(LANE, 0), giro=(-1, 0),
         rodillos=("belt_act_r1", "belt_act_r2"), velocidad_rodillos=-5.5),
    dict(numero=3, nombre="CINTA 3 - SUPERIOR", velocidad=None,
         direccion=(-1, 0), centro=(0, LANE), giro=(0, -1),
         rodillos=("belt_act_t1", "belt_act_t2"), velocidad_rodillos=-5.5),
    dict(numero=4, nombre="CINTA 4 - IZQUIERDA", velocidad=None,
         direccion=(0, -1), centro=(-LANE, 0), giro=(1, 0),
         rodillos=("belt_act_l1", "belt_act_l2"), velocidad_rodillos=5.5),
    dict(numero=5, nombre="CINTA 5 - HORNO", velocidad=BRANCH_SPEED,
         direccion=(-1, 0), centro=(0, BRANCH_Y), giro=None,
         rodillos=("belt_act_o1", "belt_act_o2"), velocidad_rodillos=-5.5),
]
# Circuito cerrado: 1 -> 2 -> 3 -> 4 -> 1. La cinta 5 no tiene siguiente.
CIRCUITO_CINTAS = [(1, 2), (2, 3), (3, 4), (4, 1)]


# =============================================================================
# CÁMARAS DE INSPECCIÓN - VISIÓN (camara_inspeccion.py)
# =============================================================================
CAMARA_UMBRAL_SATURACION = 90
CAMARA_UMBRAL_BRILLO = 70
CAMARA_AREA_MINIMA = 800
CAMARA_MARGEN_BORDE = 3
CAMARA_TIEMPO_INSPECCION = 1.5
CAMARA_NUCLEO_MORFOLOGIA = 5
CAMARA_CIRCULARIDAD_MIN = 0.85
CAMARA_EPSILON_POLIGONO = 0.04
CAMARA_PROPORCION_CUADRADO = 1.25

# Una cámara por cinta, en el mismo orden que CINTAS
NOMBRES_CAMARAS = ["cam_bottom", "cam_right", "cam_top", "cam_left", "cam_oven"]


# =============================================================================
# HORNO (horno.py)
# =============================================================================
HORNO_GEOM = "oven_roof"
HORNO_TEMPERATURA_INICIAL = 20.0
HORNO_TEMPERATURA_MINIMA = 60.0
HORNO_TEMPERATURA_MAXIMA = 220.0
HORNO_TEMPERATURA_ALTA = 150.0
HORNO_VELOCIDAD_CALENTAMIENTO = 5.0   # grados por segundo
HORNO_VELOCIDAD_ENFRIAMIENTO = 4.0
HORNO_ENFRIAMIENTO_POR_PIEZA = 5.0
HORNO_PASO_TECLADO = 10.0             # grados que suben/bajan las teclas +/-

HORNO_COLOR_FRIO = [0.1, 0.3, 1.0, 1.0]      # azul
HORNO_COLOR_TEMPLADO = [1.0, 0.6, 0.1, 1.0]  # naranja
HORNO_COLOR_CALIENTE = [1.0, 0.1, 0.1, 1.0]  # rojo


# =============================================================================
# PIEZAS (objeto_industrial.py y planta.py)
# =============================================================================
ALTURA_CINTA = BELT_TOP_Z
HOLGURA_PIEZA = OBJECT_CLEARANCE_Z
# Límites a partir de los cuales se considera que una pieza se ha caído
LIMITE_CAIDA_Z = -0.1
LIMITE_CAIDA_X = 5.0
LIMITE_CAIDA_Y = 2.4

# Piezas que crea la planta: (tipo, color RGB, masa, medidas)
#   caja     -> medidas = (medio_x, medio_y, medio_z)
#   cilindro -> medidas = (radio, medio_alto)
PIEZAS = [
    ("caja",     (0.90, 0.16, 0.12), 0.18,  (0.085, 0.085, 0.085)),
    ("cilindro", (0.12, 0.45, 0.95), 0.08,  (0.075, 0.045)),
    ("caja",     (0.95, 0.76, 0.10), 0.15,  (0.10, 0.07, 0.07)),
    ("cilindro", (0.18, 0.75, 0.30), 0.085, (0.085, 0.0375)),
    ("caja",     (0.72, 0.28, 0.90), 0.14,  (0.07, 0.11, 0.065)),
]

# Posición inicial de cada pieza al reiniciar: (x, y, número de cinta)
POSICIONES_INICIALES = [
    (-LANE, 0.95, 4),
    (LANE, -0.72, 2),
    (0.58, LANE, 3),
    (-0.62, -LANE, 1),
    (-0.45, LANE, 3),
]

# Dónde reaparecen las piezas (sobre la Cinta 1) al entrar al horno o caerse:
#   x = inicio + paso * (número de la pieza)
REAPARICION_HORNO_X0 = -1.05
REAPARICION_HORNO_PASO = 0.18
REAPARICION_CAIDA_X0 = -0.8
REAPARICION_CAIDA_PASO = 0.25


# =============================================================================
# VENTANA Y CÁMARAS OPENCV (planta.py)
# =============================================================================
MOSTRAR_VISOR = True
VENTANA = "Camaras de la planta"
ANCHO_CAMARA = 360
ALTO_CAMARA = 260
FPS_CAMARAS = 20.0
ALTO_BARRA_ESTADO = 60
CAMARA_VISOR = "overview"
