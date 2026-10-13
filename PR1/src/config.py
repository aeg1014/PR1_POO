# -----------------------------------------------------------------------------
# CONFIGURACIÓN GENERAL
# -----------------------------------------------------------------------------
SHOW_MUJOCO_VIEWER = True
SHOW_CAMERA_MOSAIC = True

CAM_W = 360
CAM_H = 260
CAM_FPS = 20.0

BELT_SPEED = 0.55          # m/s, velocidad objetivo de los objetos

# -----------------------------------------------------------------------------
# BANDERAS DE MARCHA / PARO DE CADA CINTA
# -----------------------------------------------------------------------------
# True  -> cinta en marcha
# False -> cinta detenida
# Estas variables pueden modificarse directamente antes de ejecutar el programa
# y también con las teclas 1..5 durante la simulación.
BANDERA_CINTA_1 = True      # cinta inferior (+X)
BANDERA_CINTA_2 = True      # cinta derecha (+Y)
BANDERA_CINTA_3 = True      # cinta superior (-X)
BANDERA_CINTA_4 = True      # cinta izquierda (-Y)
BANDERA_CINTA_5 = True      # cinta hacia el horno (-X)

VEL_KP = 7.0               # control de velocidad
CENTER_KP = 12.0           # mantiene el objeto centrado en la cinta
TURN_KP = 16.0             # ayuda a realizar el giro en las esquinas
MAX_FORCE = 12.0           # N por eje

BELT_HALF_LENGTH = 1.45
BELT_HALF_WIDTH = 0.28
BELT_Z = 0.12
BELT_TOP_Z = 0.18
LANE = 1.45                 # coordenada de la línea central de cada cinta
TURN_START = 1.08           # comienza a orientar el objeto hacia la siguiente cinta
CORNER_SWITCH = 1.27        # cambio efectivo de cinta dentro de la placa de esquina
RAIL_END_GAP = 0.06       # separación uniforme antes de cada zona de transferencia
RAIL_HALF_LENGTH = LANE - BELT_HALF_WIDTH - RAIL_END_GAP
RAIL_THICKNESS = 0.035
RAIL_HALF_HEIGHT = 0.10
RAIL_CENTER_Z = 0.22

# Cámaras de inspección. El poste queda fuera de la cinta, pero el cabezal
# óptico queda sobre el borde de la banda y mira casi perpendicularmente hacia
# la superficie. El FOV estrecho evita ver gran parte del suelo/planta.
CAM_HEIGHT = 0.92
CAM_FOVY = 32.0
CAM_LENS_SIDE_OFFSET = 0.10
CAM_POLE_SIDE_OFFSET = BELT_HALF_WIDTH + 0.45

# Cámara específica de la cinta del horno: el cabezal queda completamente
# FUERA del ancho de la banda y mira hacia su línea central.
CAM_OVEN_HEIGHT = 0.78
CAM_OVEN_SIDE_OFFSET = 0.46
CAM_OVEN_FOVY = 34.0

# Cámara 4 (izquierda): se coloca completamente FUERA de la banda y además
# desplazada hacia la mitad superior de la Cinta 4. De esta forma no invade
# ni la banda ni la intersección Cinta 4 -> Cinta 5/Panda.
CAM_LEFT_INSPECTION_Y = 0.82
CAM_LEFT_HEAD_CLEARANCE = 0.20
CAM_LEFT_POLE_CLEARANCE = 0.52

# Evita que las piezas roten por contacto, rozamiento o durante los giros.
# Conservamos freejoint para que sigan teniendo dinámica XYZ, pero forzamos
# quaternion identidad y velocidad angular nula después de cada paso físico.
LOCK_OBJECT_ROTATION = True

# Objetos cilíndricos del modelo (azul y verde). En tramos rectos se fija
# exactamente su coordenada transversal a la línea central de la banda.
CYLINDER_OBJECT_IDS = {1, 3}
LOCK_CYLINDERS_TO_CENTERLINE = True

# Dimensiones de las piezas. Los cilindros azul (obj1) y verde (obj3)
# conservan su radio pero tienen ahora la MITAD de la altura anterior.
OBJECT_HALF = 0.085
OBJ1_RADIUS = 0.075
OBJ1_HALF_HEIGHT = 0.045      # antes 0.090 -> altura total 0.09 m
OBJ2_HALF_Z = 0.070
OBJ3_RADIUS = 0.085
OBJ3_HALF_HEIGHT = 0.0375     # antes 0.075 -> altura total 0.075 m
OBJ4_HALF_Z = 0.065
OBJECT_CLEARANCE_Z = 0.01
OBJECT_REST_Z = [
    BELT_TOP_Z + OBJECT_HALF + OBJECT_CLEARANCE_Z,
    BELT_TOP_Z + OBJ1_HALF_HEIGHT + OBJECT_CLEARANCE_Z,
    BELT_TOP_Z + OBJ2_HALF_Z + OBJECT_CLEARANCE_Z,
    BELT_TOP_Z + OBJ3_HALF_HEIGHT + OBJECT_CLEARANCE_Z,
    BELT_TOP_Z + OBJ4_HALF_Z + OBJECT_CLEARANCE_Z,
]

# Rama lateral desde la Cinta 4 hacia el horno.
# La boca se deja completamente libre: el robot deposita la pieza sobre la
# Cinta 5 y la banda se encarga del resto. Ya no se hace un desvío dinámico
# desde la Cinta 4, que era la causa principal del atasco observado.
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

# Apertura de la barandilla interior de Cinta 4 para que el Panda pueda
# alcanzar el cubo sin atravesar una protección física.
PICK_RAIL_OPEN_Y_MIN = -0.10
PICK_RAIL_OPEN_Y_MAX = 0.72
LEFT_RAIL_UPPER_CENTER_Y = 0.5 * (RAIL_HALF_LENGTH + PICK_RAIL_OPEN_Y_MAX)
LEFT_RAIL_UPPER_HALF = 0.5 * (RAIL_HALF_LENGTH - PICK_RAIL_OPEN_Y_MAX)
LEFT_RAIL_LOWER_CENTER_Y = 0.5 * (-RAIL_HALF_LENGTH + PICK_RAIL_OPEN_Y_MIN)
LEFT_RAIL_LOWER_HALF = 0.5 * (PICK_RAIL_OPEN_Y_MIN + RAIL_HALF_LENGTH)

OVEN_CENTER_X = -4.05
OVEN_ENTRY_X = -3.58