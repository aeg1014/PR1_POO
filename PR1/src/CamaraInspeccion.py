import cv2
import numpy as np

from src.config import (
    CAMARA_UMBRAL_SATURACION, CAMARA_UMBRAL_BRILLO, CAMARA_AREA_MINIMA,
    CAMARA_MARGEN_BORDE, CAMARA_TIEMPO_INSPECCION, CAMARA_NUCLEO_MORFOLOGIA,
    CAMARA_CIRCULARIDAD_MIN, CAMARA_EPSILON_POLIGONO, CAMARA_PROPORCION_CUADRADO,
)


class CamaraInspeccion:
    """Cámara colocada encima de una cinta. Ve las piezas, reconoce su forma
    y para la cinta mientras inspecciona la pieza."""

    UMBRAL_SATURACION = CAMARA_UMBRAL_SATURACION
    UMBRAL_BRILLO = CAMARA_UMBRAL_BRILLO
    AREA_MINIMA = CAMARA_AREA_MINIMA
    MARGEN_BORDE = CAMARA_MARGEN_BORDE

    def __init__(self, nombre, titulo, cinta, tiempo_inspeccion=CAMARA_TIEMPO_INSPECCION):
        self.nombre = nombre
        self.titulo = titulo
        self.cinta = cinta
        self.tiempo_inspeccion = tiempo_inspeccion

        self.objeto_detectado = False
        self.forma_detectada = None
        self.piezas_inspeccionadas = 0

        self._contorno = None
        self._inspeccionando = False
        self._inicio_inspeccion = 0.0
        self._pieza_liberada = False

    def reiniciar(self):
        self.objeto_detectado = False
        self.forma_detectada = None
        self.piezas_inspeccionadas = 0
        self._contorno = None
        self._inspeccionando = False
        self._pieza_liberada = False

    # --- captura ---
    def capturar(self, renderer, data):
        renderer.update_scene(data, camera=self.nombre)
        rgb = renderer.render()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    # --- visión ---
    def detectar_forma(self, imagen):
        """Devuelve 'circulo', 'cuadrado', 'rectangulo', 'otra' o None si no hay pieza."""
        mascara = self._segmentar(imagen)
        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self._contorno = self._elegir_contorno(contornos, imagen.shape)
        if self._contorno is None:
            return None
        return self._clasificar(self._contorno)

    def _segmentar(self, imagen):
        # La cinta y el suelo son grises (sin color) y las piezas tienen colores vivos.
        # Se buscan píxeles con mucho color (saturación) y que no sean oscuros (brillo).
        hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
        minimo = (0, self.UMBRAL_SATURACION, self.UMBRAL_BRILLO)
        maximo = (180, 255, 255)
        mascara = cv2.inRange(hsv, minimo, maximo)
        nucleo = np.ones((CAMARA_NUCLEO_MORFOLOGIA, CAMARA_NUCLEO_MORFOLOGIA), np.uint8)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, nucleo)
        return mascara

    def _elegir_contorno(self, contornos, forma_imagen):
        alto, ancho = forma_imagen[:2]
        mejor = None
        for contorno in contornos:
            if cv2.contourArea(contorno) < self.AREA_MINIMA:
                continue
            x, y, w, h = cv2.boundingRect(contorno)
            # si toca el borde la pieza todavía no se ve entera
            if (x <= self.MARGEN_BORDE or y <= self.MARGEN_BORDE or
                    x + w >= ancho - self.MARGEN_BORDE or y + h >= alto - self.MARGEN_BORDE):
                continue
            if mejor is None or cv2.contourArea(contorno) > cv2.contourArea(mejor):
                mejor = contorno
        return mejor

    def _clasificar(self, contorno):
        area = cv2.contourArea(contorno)
        perimetro = cv2.arcLength(contorno, True)
        circularidad = 4 * np.pi * area / (perimetro * perimetro)
        aproximacion = cv2.approxPolyDP(contorno, CAMARA_EPSILON_POLIGONO * perimetro, True)
        vertices = len(aproximacion)

        if circularidad > CAMARA_CIRCULARIDAD_MIN:
            return "circulo"
        # una caja vista un poco de lado enseña también una cara lateral,
        # por eso su contorno puede tener de 4 a 6 esquinas
        if 4 <= vertices <= 6:
            (_, _), (lado1, lado2), _ = cv2.minAreaRect(contorno)
            proporcion = max(lado1, lado2) / min(lado1, lado2)
            if proporcion < CAMARA_PROPORCION_CUADRADO:
                return "cuadrado"
            return "rectangulo"
        return "otra"

    # --- comunicación con la cinta ---
    def actualizar(self, renderer, data):
        imagen = self.capturar(renderer, data)
        self.forma_detectada = self.detectar_forma(imagen)
        self.objeto_detectado = self.forma_detectada is not None
        self._controlar_cinta(data.time)
        return imagen

    def _controlar_cinta(self, tiempo):
        if not self.objeto_detectado:
            if self._inspeccionando:
                self._terminar_inspeccion()
            self._pieza_liberada = False
            return

        if self._pieza_liberada:
            return

        if not self._inspeccionando:
            self.cinta.detener()
            self._inspeccionando = True
            self._inicio_inspeccion = tiempo
            self.piezas_inspeccionadas += 1
            print(f"[{self.titulo}] pieza detectada: {self.forma_detectada} -> cinta {self.cinta.numero} parada")
        elif tiempo - self._inicio_inspeccion >= self.tiempo_inspeccion:
            self._terminar_inspeccion()
            self._pieza_liberada = True

    def _terminar_inspeccion(self):
        self._inspeccionando = False
        self.cinta.arrancar()
        print(f"[{self.titulo}] inspección terminada -> cinta {self.cinta.numero} en marcha")

    # --- dibujo ---
    def dibujar(self, imagen):
        img = imagen.copy()
        cv2.rectangle(img, (0, 0), (img.shape[1], 36), (20, 20, 20), -1)
        cv2.putText(img, self.titulo, (12, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (245, 245, 245), 2, cv2.LINE_AA)
        if self.objeto_detectado:
            cv2.drawContours(img, [self._contorno], -1, (0, 255, 0), 2)
            texto = f"DETECTADO: {self.forma_detectada}"
            color = (0, 0, 255) if self._inspeccionando else (0, 200, 0)
        else:
            texto = "sin pieza"
            color = (200, 200, 200)
        cv2.putText(img, texto, (12, img.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 2, cv2.LINE_AA)
        return img
