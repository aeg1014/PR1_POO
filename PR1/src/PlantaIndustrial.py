import time

import cv2
import mujoco
import mujoco.viewer
import numpy as np

from src import escena, robot
from src.CamaraInspeccion import CamaraInspeccion
from src.CintaTransportadora import CintaTransportadora
from src.Horno import Horno
from src.ObjetoIndustrial import Caja, Cilindro
from src.RobotPanda import RobotPanda
from src.config import (
    ROBOT_CONFIG, LANE, OVEN_ENTRY_X,
    CINTAS, CIRCUITO_CINTAS, BELT_SPEED, INDICE_CINTA_HORNO,
    NOMBRES_CAMARAS, PIEZAS, POSICIONES_INICIALES,
    HORNO_GEOM, HORNO_PASO_TECLADO,
    REAPARICION_HORNO_X0, REAPARICION_HORNO_PASO,
    REAPARICION_CAIDA_X0, REAPARICION_CAIDA_PASO,
    SHOW_MUJOCO_VIEWER, ROBOT_IK_TEST_MODE,
    VENTANA, CAM_W, CAM_H, CAM_FPS, ALTO_BARRA_ESTADO,
    CAMARA_VISOR, OUTPUT_DIR,
)


class PlantaIndustrial:
    """Coordina todos los elementos de la planta y contiene el bucle principal."""

    VENTANA = VENTANA
    CAM_W = CAM_W
    CAM_H = CAM_H
    CAM_FPS = CAM_FPS
    BELT_SPEED = BELT_SPEED

    def __init__(self, mostrar_visor=SHOW_MUJOCO_VIEWER, modo_prueba_ik=ROBOT_IK_TEST_MODE):
        self.mostrar_visor = mostrar_visor

        self.cintas = self._crear_cintas()
        self.cinta_horno = self.cintas[INDICE_CINTA_HORNO]
        self.camaras = self._crear_camaras()
        self.objetos = self._crear_objetos()
        self.horno = Horno(HORNO_GEOM, OVEN_ENTRY_X)
        self.robot = RobotPanda(ROBOT_CONFIG, modo_prueba_ik)

        self.model = None
        self.data = None
        self.renderer = None
        self.visor = None

        self.pausada = False
        self.en_marcha = False
        self.mosaico = None
        self._ultimo_render = -1.0
        self.carpeta_capturas = OUTPUT_DIR

    # ------------------------------------------------------------------
    # Creación de los elementos
    # ------------------------------------------------------------------
    def _crear_cintas(self):
        cintas = []
        for datos in CINTAS:
            datos = dict(datos)
            if datos["velocidad"] is None:
                datos["velocidad"] = self.BELT_SPEED
            cintas.append(CintaTransportadora(**datos))
        # circuito cerrado: 1 -> 2 -> 3 -> 4 -> 1. La cinta 5 no tiene siguiente.
        for origen, destino in CIRCUITO_CINTAS:
            cintas[origen - 1].siguiente = cintas[destino - 1]
        return cintas

    def _crear_camaras(self):
        camaras = []
        for nombre, cinta in zip(NOMBRES_CAMARAS, self.cintas):
            camaras.append(CamaraInspeccion(nombre, cinta.nombre, cinta))
        return camaras

    def _crear_objetos(self):
        objetos = []
        for numero, (tipo, color, masa, medidas) in enumerate(PIEZAS):
            if tipo == "caja":
                objetos.append(Caja(numero, color, masa, *medidas))
            elif tipo == "cilindro":
                objetos.append(Cilindro(numero, color, masa, *medidas))
            else:
                raise ValueError(f"Tipo de pieza desconocido: {tipo}")
        return objetos

    # ------------------------------------------------------------------
    # Inicialización
    # ------------------------------------------------------------------
    def inicializar(self):
        xml_objetos = "\n".join(obj.generar_xml() for obj in self.objetos)
        xml = escena.construir_mjcf(xml_objetos)
        self.model = robot.build_model_with_franka(xml, ROBOT_CONFIG)
        self.data = mujoco.MjData(self.model)

        for cinta in self.cintas:
            cinta.conectar(self.model)
        for objeto in self.objetos:
            objeto.conectar(self.model)
        self.horno.conectar(self.model)
        self.robot.conectar(self.model, self.data)

        self.reiniciar()

        self.renderer = mujoco.Renderer(self.model, height=self.CAM_H, width=self.CAM_W)
        cv2.namedWindow(self.VENTANA, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.VENTANA, self.CAM_W * 3, self.CAM_H * 2 + ALTO_BARRA_ESTADO)

        if self.mostrar_visor:
            self.visor = mujoco.viewer.launch_passive(self.model, self.data)
            self.visor.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.visor.cam.fixedcamid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, CAMARA_VISOR)

        self.en_marcha = True

    def reiniciar(self):
        for objeto, (x, y, numero_cinta) in zip(self.objetos, POSICIONES_INICIALES):
            objeto.colocar(self.data, x, y, self.cintas[numero_cinta - 1])
        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

        self.robot.reiniciar(self.model, self.data)
        for cinta in self.cintas:
            cinta.arrancar()
            cinta.actualizar_rodillos(self.data)
        for camara in self.camaras:
            camara.reiniciar()
        self.horno.reiniciar()
        self.horno.actualizar_color(self.model)
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------
    # Un paso de simulación
    # ------------------------------------------------------------------
    def actualizar(self):
        if self.pausada:
            return
        self.robot.antes_del_paso(self.model, self.data, self.objetos, self.cintas)
        self._aplicar_transporte()
        self.robot.controlar(self.model, self.data)

        mujoco.mj_step(self.model, self.data)

        for objeto in self.objetos:
            objeto.estabilizar(self.data)
        self.robot.despues_del_paso(self.model, self.data)

        self.horno.actualizar(self.model.opt.timestep)
        self.horno.actualizar_color(self.model)
        for cinta in self.cintas:
            cinta.actualizar_rodillos(self.data)
        mujoco.mj_forward(self.model, self.data)

    def _aplicar_transporte(self):
        self.data.xfrc_applied[:] = 0.0
        for objeto in self.objetos:
            if objeto.numero == self.robot.pieza_agarrada():
                continue

            pos = objeto.posicion(self.data)
            vel = objeto.velocidad(self.data)
            if objeto.numero == self.robot.pieza_reservada():
                fx, fy = self.robot.fuerza_retencion(pos, vel, CintaTransportadora.MAX_FORCE)
            else:
                objeto.actualizar_cinta(self.data)
                fx, fy = objeto.cinta.calcular_fuerza(pos, vel)
            objeto.aplicar_fuerza(self.data, fx, fy)

            if objeto.cinta is self.cinta_horno and self.horno.detectar_entrada(pos):
                self.horno.registrar_pieza()
                print(f"[Horno] entra {objeto} -> total {self.horno.numero_piezas}")
                x = REAPARICION_HORNO_X0 + REAPARICION_HORNO_PASO * (objeto.numero % 3)
                objeto.colocar(self.data, x, -LANE, self.cintas[0])
            elif objeto.se_ha_caido(self.data):
                x = REAPARICION_CAIDA_X0 + REAPARICION_CAIDA_PASO * objeto.numero
                objeto.colocar(self.data, x, -LANE, self.cintas[0])

    def enviar_pieza_al_horno(self):
        for objeto in self.objetos:
            if objeto.cinta is self.cintas[3] and objeto.numero != self.robot.pieza_agarrada():
                cfg = ROBOT_CONFIG
                objeto.colocar(self.data, cfg["place_x"], cfg["place_y"], self.cinta_horno)
                mujoco.mj_forward(self.model, self.data)
                print(f"{objeto} enviado a la Cinta 5")
                return
        print("No hay ninguna pieza en la Cinta 4")

    # ------------------------------------------------------------------
    # Cámaras y ventana OpenCV
    # ------------------------------------------------------------------
    def _toca_actualizar_camaras(self):
        if self.pausada or self._ultimo_render < 0.0:
            return True
        return self.data.time - self._ultimo_render >= 1.0 / self.CAM_FPS

    def actualizar_camaras(self):
        imagenes = []
        for camara in self.camaras:
            imagen = camara.actualizar(self.renderer, self.data)
            imagenes.append(camara.dibujar(imagen))
        imagenes.append(self._vista_general())
        self.mosaico = self._montar_mosaico(imagenes)
        self._ultimo_render = self.data.time

    def _vista_general(self):
        self.renderer.update_scene(self.data, camera=CAMARA_VISOR)
        imagen = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
        cv2.rectangle(imagen, (0, 0), (imagen.shape[1], 36), (20, 20, 20), -1)
        cv2.putText(imagen, "VISTA GENERAL", (12, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (245, 245, 245), 2, cv2.LINE_AA)
        return imagen

    def _montar_mosaico(self, imagenes):
        arriba = np.hstack(imagenes[0:3])
        abajo = np.hstack(imagenes[3:6])
        barra = np.full((ALTO_BARRA_ESTADO, arriba.shape[1], 3), 20, dtype=np.uint8)

        estado_cintas = " ".join(str(cinta) for cinta in self.cintas)
        linea1 = (f"t={self.data.time:6.1f}s  {'PAUSA' if self.pausada else 'EN MARCHA'}  "
                  f"{estado_cintas}  {self.horno}  {self.robot.texto_estado()}")
        linea2 = "Q salir | P pausa | 1-5 cintas | R reset | S captura | O horno | +/- temp | E pieza al horno"
        cv2.putText(barra, linea1, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(barra, linea2, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 220, 255), 1, cv2.LINE_AA)
        return np.vstack((arriba, abajo, barra))

    # ------------------------------------------------------------------
    # Teclado
    # ------------------------------------------------------------------
    def procesar_teclado(self, tecla):
        if tecla in (ord('q'), 27):
            self.en_marcha = False
        elif tecla == ord('p'):
            self.pausada = not self.pausada
            print("PAUSA" if self.pausada else "SIMULACION REANUDADA")
        elif ord('1') <= tecla <= ord('5'):
            cinta = self.cintas[tecla - ord('1')]
            activa = cinta.conmutar()
            print(f"Cinta {cinta.numero}: {'ON' if activa else 'OFF'}")
        elif tecla == ord('r'):
            self.reiniciar()
            print("Planta reiniciada")
        elif tecla == ord('s'):
            self.guardar_captura()
        elif tecla == ord('o'):
            encendido = self.horno.conmutar()
            print(f"Horno {'encendido' if encendido else 'apagado'}")
        elif tecla == ord('+'):
            self.horno.aumentar_temperatura(HORNO_PASO_TECLADO)
            print(self.horno)
        elif tecla == ord('-'):
            self.horno.disminuir_temperatura(HORNO_PASO_TECLADO)
            print(self.horno)
        elif tecla == ord('e'):
            self.enviar_pieza_al_horno()
        elif tecla == ord('t') and self.robot.modo_prueba_ik:
            self.robot.ir_a_objetivo(self.model)
            print("[IK] Objetivo:", self.robot.texto_objetivo())
        elif tecla == ord('h') and self.robot.modo_prueba_ik:
            self.robot.ir_a_home(self.model)
            print("[IK] Objetivo HOME")

    def guardar_captura(self):
        if self.mosaico is None:
            return
        self.carpeta_capturas.mkdir(parents=True, exist_ok=True)
        archivo = self.carpeta_capturas / f"mosaico_{int(time.time())}.png"
        cv2.imwrite(str(archivo), self.mosaico)
        print(f"Captura guardada: {archivo}")

    # ------------------------------------------------------------------
    # Bucle principal
    # ------------------------------------------------------------------
    def ejecutar(self):
        self.inicializar()
        self._mostrar_ayuda()
        try:
            while self.en_marcha:
                if self.visor is not None and not self.visor.is_running():
                    break
                inicio = time.perf_counter()

                self.actualizar()
                if self.visor is not None:
                    self.visor.sync()
                if self._toca_actualizar_camaras():
                    self.actualizar_camaras()
                    cv2.imshow(self.VENTANA, self.mosaico)
                self.procesar_teclado(cv2.waitKey(1) & 0xFF)

                espera = self.model.opt.timestep - (time.perf_counter() - inicio)
                if espera > 0:
                    time.sleep(espera)
        finally:
            self.cerrar()

    def cerrar(self):
        if self.visor is not None:
            self.visor.close()
        if self.renderer is not None:
            self.renderer.close()
        cv2.destroyAllWindows()

    def _mostrar_ayuda(self):
        print("\n=== PLANTA INDUSTRIAL ===")
        print("Circuito: Cinta 1 -> 2 -> 3 -> 4 -> 1. Cinta 5 lleva las piezas al horno.")
        print("Q/ESC salir | P pausa | 1..5 cintas ON/OFF | R reset | S captura")
        print("O horno ON/OFF | +/- temperatura | E manda una pieza de la Cinta 4 al horno")
        if self.robot.modo_prueba_ik:
            print("Robot en modo prueba IK: T objetivo | H home")
            print("Objetivo IK:", self.robot.texto_objetivo())
        else:
            print("Robot en modo pick-and-place automatico")
        print()

if __name__ == "__main__":
    planta = PlantaIndustrial()
    planta.ejecutar()
    planta.cerrar()
    planta._mostrar_ayuda()