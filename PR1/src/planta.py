import time
from pathlib import Path

import cv2
import mujoco
import mujoco.viewer
import numpy as np

from src import escena, robot
from src.camara_inspeccion import CamaraInspeccion
from src.cinta_transportadora import CintaTransportadora
from src.horno import Horno
from src.objeto_industrial import Caja, Cilindro
from src.robot_panda import RobotPanda



class PlantaIndustrial:
    """Coordina todos los elementos de la planta y contiene el bucle principal."""

    VENTANA = "Camaras de la planta"
    ANCHO_CAMARA = 360
    ALTO_CAMARA = 260
    FPS_CAMARAS = 20.0
    VELOCIDAD_CINTAS = 0.55

    def __init__(self, mostrar_visor=True, modo_prueba_ik=True):
        self.mostrar_visor = mostrar_visor

        self.cintas = self._crear_cintas()
        self.cinta_horno = self.cintas[4]
        self.camaras = self._crear_camaras()
        self.objetos = self._crear_objetos()
        self.horno = Horno("oven_roof", escena.OVEN_ENTRY_X)
        self.robot = RobotPanda(escena.CONFIG_ROBOT, modo_prueba_ik)

        self.model = None
        self.data = None
        self.renderer = None
        self.visor = None

        self.pausada = False
        self.en_marcha = False
        self.mosaico = None
        self._ultimo_render = -1.0
        self.carpeta_capturas = Path(__file__).resolve().parent.parent / "capturas"

    # ------------------------------------------------------------------
    # Creación de los elementos
    # ------------------------------------------------------------------
    def _crear_cintas(self):
        lane = escena.LANE
        v = self.VELOCIDAD_CINTAS
        cinta1 = CintaTransportadora(1, "CINTA 1 - INFERIOR", v, (1, 0), (0, -lane), giro=(0, 1),
                                     rodillos=("belt_act_b1", "belt_act_b2"), velocidad_rodillos=5.5)
        cinta2 = CintaTransportadora(2, "CINTA 2 - DERECHA", v, (0, 1), (lane, 0), giro=(-1, 0),
                                     rodillos=("belt_act_r1", "belt_act_r2"), velocidad_rodillos=-5.5)
        cinta3 = CintaTransportadora(3, "CINTA 3 - SUPERIOR", v, (-1, 0), (0, lane), giro=(0, -1),
                                     rodillos=("belt_act_t1", "belt_act_t2"), velocidad_rodillos=-5.5)
        cinta4 = CintaTransportadora(4, "CINTA 4 - IZQUIERDA", v, (0, -1), (-lane, 0), giro=(1, 0),
                                     rodillos=("belt_act_l1", "belt_act_l2"), velocidad_rodillos=5.5)
        cinta5 = CintaTransportadora(5, "CINTA 5 - HORNO", escena.BRANCH_SPEED, (-1, 0),
                                     (0, escena.BRANCH_Y),
                                     rodillos=("belt_act_o1", "belt_act_o2"), velocidad_rodillos=-5.5)
        # circuito cerrado: 1 -> 2 -> 3 -> 4 -> 1. La cinta 5 no tiene siguiente.
        cinta1.siguiente = cinta2
        cinta2.siguiente = cinta3
        cinta3.siguiente = cinta4
        cinta4.siguiente = cinta1
        return [cinta1, cinta2, cinta3, cinta4, cinta5]

    def _crear_camaras(self):
        nombres = ["cam_bottom", "cam_right", "cam_top", "cam_left", "cam_oven"]
        camaras = []
        for nombre, cinta in zip(nombres, self.cintas):
            camaras.append(CamaraInspeccion(nombre, cinta.nombre, cinta))
        return camaras

    def _crear_objetos(self):
        return [
            Caja(0, (0.90, 0.16, 0.12), 0.18, 0.085, 0.085, 0.085),
            Cilindro(1, (0.12, 0.45, 0.95), 0.08, 0.075, 0.045),
            Caja(2, (0.95, 0.76, 0.10), 0.15, 0.10, 0.07, 0.07),
            Cilindro(3, (0.18, 0.75, 0.30), 0.085, 0.085, 0.0375),
            Caja(4, (0.50, 0.12, 0.80), 0.14, 0.07, 0.11, 0.065),
        ]

    # ------------------------------------------------------------------
    # Inicialización
    # ------------------------------------------------------------------
    def inicializar(self):
        xml_objetos = "\n".join(obj.generar_xml() for obj in self.objetos)
        xml = escena.construir_mjcf(xml_objetos)
        self.model = robot.build_model_with_franka(xml, escena.CONFIG_ROBOT)
        self.data = mujoco.MjData(self.model)

        for cinta in self.cintas:
            cinta.conectar(self.model)
        for objeto in self.objetos:
            objeto.conectar(self.model)
        self.horno.conectar(self.model)
        self.robot.conectar(self.model, self.data)

        self.reiniciar()

        self.renderer = mujoco.Renderer(self.model, height=self.ALTO_CAMARA, width=self.ANCHO_CAMARA)
        cv2.namedWindow(self.VENTANA, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.VENTANA, self.ANCHO_CAMARA * 3, self.ALTO_CAMARA * 2 + 60)

        if self.mostrar_visor:
            self.visor = mujoco.viewer.launch_passive(self.model, self.data)
            self.visor.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.visor.cam.fixedcamid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")

        self.en_marcha = True

    def reiniciar(self):
        lane = escena.LANE
        posiciones = [(-lane, 0.95, 4), (lane, -0.72, 2), (0.58, lane, 3),
                      (-0.62, -lane, 1), (-0.45, lane, 3)]
        for objeto, (x, y, numero_cinta) in zip(self.objetos, posiciones):
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
                fx, fy = self.robot.fuerza_retencion(pos, vel, CintaTransportadora.FUERZA_MAXIMA)
            else:
                objeto.actualizar_cinta(self.data)
                fx, fy = objeto.cinta.calcular_fuerza(pos, vel)
            objeto.aplicar_fuerza(self.data, fx, fy)

            if objeto.cinta is self.cinta_horno and self.horno.detectar_entrada(pos):
                self.horno.registrar_pieza()
                print(f"[Horno] entra {objeto} -> total {self.horno.numero_piezas}")
                x = -1.05 + 0.18 * (objeto.numero % 3)
                objeto.colocar(self.data, x, -escena.LANE, self.cintas[0])
            elif objeto.se_ha_caido(self.data):
                x = -0.8 + 0.25 * objeto.numero
                objeto.colocar(self.data, x, -escena.LANE, self.cintas[0])

    def enviar_pieza_al_horno(self):
        """Atajo para la demostración: hace lo mismo que la descarga del robot,
        pone una pieza de la Cinta 4 al principio de la Cinta 5."""
        for objeto in self.objetos:
            if objeto.cinta is self.cintas[3] and objeto.numero != self.robot.pieza_agarrada():
                cfg = escena.CONFIG_ROBOT
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
        return self.data.time - self._ultimo_render >= 1.0 / self.FPS_CAMARAS

    def actualizar_camaras(self):
        imagenes = []
        for camara in self.camaras:
            imagen = camara.actualizar(self.renderer, self.data)
            imagenes.append(camara.dibujar(imagen))
        imagenes.append(self._vista_general())
        self.mosaico = self._montar_mosaico(imagenes)
        self._ultimo_render = self.data.time

    def _vista_general(self):
        self.renderer.update_scene(self.data, camera="overview")
        imagen = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
        cv2.rectangle(imagen, (0, 0), (imagen.shape[1], 36), (20, 20, 20), -1)
        cv2.putText(imagen, "VISTA GENERAL", (12, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (245, 245, 245), 2, cv2.LINE_AA)
        return imagen

    def _montar_mosaico(self, imagenes):
        arriba = np.hstack(imagenes[0:3])
        abajo = np.hstack(imagenes[3:6])
        barra = np.full((60, arriba.shape[1], 3), 20, dtype=np.uint8)

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
            self.horno.aumentar_temperatura(10.0)
            print(self.horno)
        elif tecla == ord('-'):
            self.horno.disminuir_temperatura(10.0)
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
        print("\n=== PLANTA INDUSTRIAL (version POO) ===")
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