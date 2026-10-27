import mujoco

from src.config import (
    HORNO_TEMPERATURA_INICIAL, HORNO_TEMPERATURA_MINIMA, HORNO_TEMPERATURA_MAXIMA,
    HORNO_TEMPERATURA_ALTA, HORNO_VELOCIDAD_CALENTAMIENTO, HORNO_VELOCIDAD_ENFRIAMIENTO,
    HORNO_ENFRIAMIENTO_POR_PIEZA, HORNO_COLOR_FRIO, HORNO_COLOR_TEMPLADO, HORNO_COLOR_CALIENTE,
)


class Horno:
    """Horno del final de la Cinta 5. Cuenta piezas y controla su temperatura."""

    AZUL = HORNO_COLOR_FRIO
    NARANJA = HORNO_COLOR_TEMPLADO
    ROJO = HORNO_COLOR_CALIENTE

    def __init__(self, nombre_geom, entrada_x, temperatura_inicial=HORNO_TEMPERATURA_INICIAL,
                 temperatura_minima=HORNO_TEMPERATURA_MINIMA,
                 temperatura_maxima=HORNO_TEMPERATURA_MAXIMA):
        self.nombre_geom = nombre_geom
        self.geom_id = -1
        self.entrada_x = entrada_x

        self.temperatura_ambiente = temperatura_inicial
        self.temperatura = temperatura_inicial
        self.temperatura_minima = temperatura_minima
        self.temperatura_maxima = temperatura_maxima
        self.temperatura_alta = HORNO_TEMPERATURA_ALTA
        self.velocidad_calentamiento = HORNO_VELOCIDAD_CALENTAMIENTO   # grados por segundo
        self.velocidad_enfriamiento = HORNO_VELOCIDAD_ENFRIAMIENTO
        self.enfriamiento_por_pieza = HORNO_ENFRIAMIENTO_POR_PIEZA

        self.encendido = True
        self.numero_piezas = 0
        self.pieza_nueva = False

    def conectar(self, model):
        self.geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, self.nombre_geom)
        if self.geom_id < 0:
            raise RuntimeError(f"No existe el geom '{self.nombre_geom}' del horno")

    def reiniciar(self):
        self.temperatura = self.temperatura_ambiente
        self.encendido = True
        self.numero_piezas = 0
        self.pieza_nueva = False

    @property
    def estado(self):
        if not self.encendido:
            return "APAGADO"
        if self.temperatura < self.temperatura_minima:
            return "CALENTANDO"
        return "LISTO"

    # --- piezas ---
    def detectar_entrada(self, posicion):
        """True si la pieza ya ha pasado la boca del horno."""
        self.pieza_nueva = posicion[0] <= self.entrada_x
        return self.pieza_nueva

    def registrar_pieza(self):
        self.numero_piezas += 1
        # la pieza entra fría y le quita algo de calor al horno
        self.disminuir_temperatura(self.enfriamiento_por_pieza)

    # --- temperatura ---
    def encender(self):
        self.encendido = True

    def apagar(self):
        self.encendido = False

    def conmutar(self):
        self.encendido = not self.encendido
        return self.encendido

    def aumentar_temperatura(self, incremento):
        self.temperatura = min(self.temperatura + incremento, self.temperatura_maxima)

    def disminuir_temperatura(self, decremento):
        self.temperatura = max(self.temperatura - decremento, self.temperatura_ambiente)

    def obtener_temperatura(self):
        return self.temperatura

    def actualizar(self, dt):
        """Se llama en cada paso de la simulación."""
        if self.encendido:
            self.aumentar_temperatura(self.velocidad_calentamiento * dt)
        else:
            self.disminuir_temperatura(self.velocidad_enfriamiento * dt)

    def actualizar_color(self, model):
        if self.temperatura < self.temperatura_minima:
            color = self.AZUL
        elif self.temperatura < self.temperatura_alta:
            color = self.NARANJA
        else:
            color = self.ROJO
        model.geom_rgba[self.geom_id] = color

    def __str__(self):
        return f"Horno {self.temperatura:.0f}C {self.estado} piezas={self.numero_piezas}"
