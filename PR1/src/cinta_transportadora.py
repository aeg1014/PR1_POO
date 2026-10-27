import mujoco
import numpy as np



class CintaTransportadora:
    """Una cinta de la planta. Sabe si está en marcha y cómo mover las piezas."""

    # Atributos de clase: son iguales para todas las cintas
    KP_VELOCIDAD = 7.0
    KP_CENTRADO = 12.0
    FUERZA_MAXIMA = 12.0
    INICIO_GIRO = 1.08
    CAMBIO_CINTA = 1.27

    def __init__(self, numero, nombre, velocidad, direccion, centro, giro=None,
                 rodillos=(), velocidad_rodillos=5.5):
        self.numero = numero
        self.nombre = nombre
        self.velocidad = velocidad
        self.direccion = np.array(direccion, dtype=float)
        self.centro = np.array(centro, dtype=float)
        self.giro = None if giro is None else np.array(giro, dtype=float)
        self.siguiente = None
        self.activa = True

        self._nombres_rodillos = rodillos
        self._velocidad_rodillos = velocidad_rodillos
        self._ids_rodillos = []

    @property
    def velocidad(self):
        return self._velocidad

    @velocidad.setter
    def velocidad(self, valor):
        if valor < 0:
            raise ValueError("La velocidad de la cinta no puede ser negativa")
        self._velocidad = valor

    def __str__(self):
        estado = "ON" if self.activa else "OFF"
        return f"C{self.numero}:{estado}"

    # --- marcha / paro ---
    def arrancar(self):
        self.activa = True

    def detener(self):
        self.activa = False

    def conmutar(self):
        self.activa = not self.activa
        return self.activa

    def esta_activa(self):
        return self.activa

    # --- conexión con MuJoCo ---
    def conectar(self, model):
        self._ids_rodillos = []
        for nombre in self._nombres_rodillos:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, nombre)
            if aid < 0:
                raise RuntimeError(f"No se encontró el actuador '{nombre}'")
            self._ids_rodillos.append(aid)

    def actualizar_rodillos(self, data):
        valor = self._velocidad_rodillos if self.activa else 0.0
        for aid in self._ids_rodillos:
            data.ctrl[aid] = valor

    # --- transporte de piezas ---
    def avance(self, pos):
        """Distancia recorrida a lo largo de la cinta (proyección sobre la dirección)."""
        return float(np.dot(pos[:2], self.direccion))

    def ha_terminado(self, pos):
        return self.siguiente is not None and self.avance(pos) >= self.CAMBIO_CINTA

    def en_tramo_recto(self, pos):
        return self.giro is None or self.avance(pos) < self.INICIO_GIRO

    def eje_transversal(self):
        """0 si la pieza se centra en X, 1 si se centra en Y."""
        if abs(self.direccion[0]) > 0.5:
            return 1
        return 0

    def calcular_fuerza(self, pos, vel):
        pos = np.array(pos[:2], dtype=float)
        vel = np.array(vel[:2], dtype=float)

        if not self.activa:
            fuerza = self.KP_VELOCIDAD * (0.0 - vel)
            return self._limitar(fuerza)

        # alpha va de 0 (tramo recto) a 1 (ya en la esquina)
        alpha = 0.0
        if self.giro is not None:
            alpha = (self.avance(pos) - self.INICIO_GIRO) / (self.CAMBIO_CINTA - self.INICIO_GIRO)
            alpha = float(np.clip(alpha, 0.0, 1.0))

        vel_deseada = self.velocidad * (1.0 - 0.45 * alpha) * self.direccion
        if self.giro is not None:
            vel_deseada = vel_deseada + self.velocidad * 0.85 * alpha * self.giro

        # error lateral: lo que se separa la pieza de la línea central
        error = self.centro - pos
        error_lateral = error - np.dot(error, self.direccion) * self.direccion

        fuerza = self.KP_VELOCIDAD * (vel_deseada - vel)
        fuerza = fuerza + (1.0 - alpha) * self.KP_CENTRADO * error_lateral
        return self._limitar(fuerza)

    def _limitar(self, fuerza):
        fx = float(np.clip(fuerza[0], -self.FUERZA_MAXIMA, self.FUERZA_MAXIMA))
        fy = float(np.clip(fuerza[1], -self.FUERZA_MAXIMA, self.FUERZA_MAXIMA))
        return fx, fy
