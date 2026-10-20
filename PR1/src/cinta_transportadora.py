import numpy as np

from src.config import *


class CintaTransportadora:

    def __init__(self, id, nombre, velocidad, direccion, actuadores, sentido_rodillo=1.0):
        self.id = id
        self.nombre = nombre
        self.velocidad = velocidad
        self.direccion = np.array(direccion, dtype=float)
        self.activa = True
        self.actuadores = actuadores
        self.sentido_rodillo = sentido_rodillo
        self.vel_rodillo = 5.5

    def arrancar(self):
        self.activa = True

    def detener(self):
        self.activa = False

    def conmutar(self):
        self.activa = not self.activa
        return self.activa

    def esta_activa(self):
        return self.activa

    def _limitar(self, fx, fy):
        return (float(np.clip(fx, -MAX_FORCE, MAX_FORCE)),
                float(np.clip(fy, -MAX_FORCE, MAX_FORCE)))

    def cinta_activa(self):
        """Devuelve la bandera de marcha de la cinta asociada al segmento 0..4."""
        if self.id == 0:
            return BANDERA_CINTA_1
        if self.id == 1:
            return BANDERA_CINTA_2
        if self.id == 2:
            return BANDERA_CINTA_3
        if self.id == 3:
            return BANDERA_CINTA_4
        if self.id == 4:
            return BANDERA_CINTA_5
        return False

    def conmutar_cinta(self,numero):
        """Conmuta en tiempo de ejecución una bandera de marcha/parada."""
        global BANDERA_CINTA_1, BANDERA_CINTA_2, BANDERA_CINTA_3
        global BANDERA_CINTA_4, BANDERA_CINTA_5

        if numero == 1:
            BANDERA_CINTA_1 = not BANDERA_CINTA_1
            return BANDERA_CINTA_1
        if numero == 2:
            BANDERA_CINTA_2 = not BANDERA_CINTA_2
            return BANDERA_CINTA_2
        if numero == 3:
            BANDERA_CINTA_3 = not BANDERA_CINTA_3
            return BANDERA_CINTA_3
        if numero == 4:
            BANDERA_CINTA_4 = not BANDERA_CINTA_4
            return BANDERA_CINTA_4
        if numero == 5:
            BANDERA_CINTA_5 = not BANDERA_CINTA_5
            return BANDERA_CINTA_5
        raise ValueError(f"Número de cinta inválido: {numero}")

    def fuerza_transporte(self, pos, vel):
        x, y = float(pos[0]), float(pos[1])
        vx, vy = float(vel[0]), float(vel[1])

        # cinta parada -> frenar la pieza
        if not self.activa:
            return self._limitar(VEL_KP * (0.0 - vx), VEL_KP * (0.0 - vy))

        fx = 0.0
        fy = 0.0

        if self.id == 0:
            alpha = np.clip((x - TURN_START) / max(CORNER_SWITCH - TURN_START, 1e-6), 0.0, 1.0)
            desired_vx = self.velocidad * (1.0 - 0.45 * alpha)
            desired_vy = self.velocidad * (0.85 * alpha)
            fx = VEL_KP * (desired_vx - vx)
            fy = VEL_KP * (desired_vy - vy) + (1.0 - alpha) * CENTER_KP * (-LANE - y)

        elif self.id == 1:  # derecha: +Y -> -X
            alpha = np.clip((y - TURN_START) / max(CORNER_SWITCH - TURN_START, 1e-6), 0.0, 1.0)
            desired_vx = -BELT_SPEED * (0.85 * alpha)
            desired_vy = BELT_SPEED * (1.0 - 0.45 * alpha)
            fx = VEL_KP * (desired_vx - vx) + (1.0 - alpha) * CENTER_KP * (LANE - x)
            fy = VEL_KP * (desired_vy - vy)

        elif self.id == 2:  # superior: -X -> -Y
            alpha = np.clip((-x - TURN_START) / max(CORNER_SWITCH - TURN_START, 1e-6), 0.0, 1.0)
            desired_vx = -BELT_SPEED * (1.0 - 0.45 * alpha)
            desired_vy = -BELT_SPEED * (0.85 * alpha)
            fx = VEL_KP * (desired_vx - vx)
            fy = VEL_KP * (desired_vy - vy) + (1.0 - alpha) * CENTER_KP * (LANE - y)

        elif self.id == 3:  # izquierda: -Y -> +X en esquina inferior
            alpha = np.clip((-y - TURN_START) / max(CORNER_SWITCH - TURN_START, 1e-6), 0.0, 1.0)
            desired_vx = BELT_SPEED * (0.85 * alpha)
            desired_vy = -BELT_SPEED * (1.0 - 0.45 * alpha)
            fx = VEL_KP * (desired_vx - vx) + (1.0 - alpha) * CENTER_KP * (-LANE - x)
            fy = VEL_KP * (desired_vy - vy)

        elif self.id == 4:
            fx = VEL_KP * (-self.velocidad - vx)
            fy = VEL_KP * (0.0 - vy) + CENTER_KP * (BRANCH_Y - y)

        return self._limitar(fx, fy)

    def actualizar_rodillos(self, model, data, pos):
        """Máquina de estados del circuito. La Cinta 5 solo la activa el robot."""
        x, y = float(pos[0]), float(pos[1])

        if self.id == 0 and x >= CORNER_SWITCH:
            return 1
        if self.id == 1 and y >= CORNER_SWITCH:
            return 2
        if self.id == 2 and x <= -CORNER_SWITCH:
            return 3
        if self.id == 3 and y <= -CORNER_SWITCH:
            return 0
        return self.id



