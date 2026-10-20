import time

import cv2
import numpy as np
import mujoco
import mujoco.viewer

from src.cinta_transportadora import CintaTransportadora
from src.camara_inspeccion import CamaraInspeccion
from src.objeto_industrial import Caja, Cilindro, ObjetoIndustrial
from src.horno import Horno
from src.config import *


class PlantaIndustrial:

    def __init__(self):
        self.model = None
        self.data = None
        self.renderer = None
        self.viewer = None
        self.pausa = False
        self.corriendo = True
        self.objeto_industrial = ObjetoIndustrial(id=0, forma="CAJA", segmento="SEGMENTO", z_reposo=0.0)

        self.cintas = [
            CintaTransportadora(0, "Cinta 1", BELT_SPEED, [1, 0], ["belt_act_b1", "belt_act_b2"], 1.0),
            CintaTransportadora(1, "Cinta 2", BELT_SPEED, [0, 1], ["belt_act_r1", "belt_act_r2"], -1.0),
            CintaTransportadora(2, "Cinta 3", BELT_SPEED, [-1, 0], ["belt_act_t1", "belt_act_t2"], -1.0),
            CintaTransportadora(3, "Cinta 4", BELT_SPEED, [0, -1], ["belt_act_l1", "belt_act_l2"], 1.0),
            CintaTransportadora(4, "Cinta 5", BRANCH_SPEED, [-1, 0], ["belt_act_o1", "belt_act_o2"], -1.0),
        ]

        self.camaras = [
            CamaraInspeccion("cam_bottom", "CINTA 1 - INFERIOR", self.cintas[0]),
            CamaraInspeccion("cam_right", "CINTA 2 - DERECHA", self.cintas[1]),
            CamaraInspeccion("cam_top", "CINTA 3 - SUPERIOR", self.cintas[2]),
            CamaraInspeccion("cam_left", "CINTA 4 - IZQUIERDA", self.cintas[3]),
            CamaraInspeccion("cam_oven", "CINTA 5 - HORNO", self.cintas[4]),
            CamaraInspeccion("overview", "VISTA GENERAL"),
        ]

        self.objetos = [
            Caja(0, 3, OBJECT_REST_Z[0]),
            Cilindro(1, 1, OBJECT_REST_Z[1]),
            Caja(2, 2, OBJECT_REST_Z[2]),
            Cilindro(3, 3, OBJECT_REST_Z[3]),
            Caja(4, 0, OBJECT_REST_Z[4]),
        ]

        self.horno = Horno()

    def inicializar(self):

        self.data = mujoco.MjData(self.model)

        for obj in self.objetos:
            obj.get_ids(self.model)

        self.reiniciar_objetos()

        self.renderer = mujoco.Renderer(self.model, height=CAM_H, width=CAM_W)
        cv2.namedWindow("Camaras de la planta", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camaras de la planta", CAM_W * 3, CAM_H * 2)

    def reiniciar_objetos(self):
        inicios = [
            (-LANE, 0.95, 3),
            (LANE, -0.72, 1),
            (0.58, LANE, 2),
            (-0.62, -LANE, 0),
            (-0.45, LANE, 2),
        ]
        for obj, (x, y, seg) in zip(self.objetos, inicios):
            obj.colocar(self.data, x, y, seg)

        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def aplicar_transporte(self, segments, model, data, dof_addrs):
        self.data.xfrc_applied[:] = 0.0

        for i, obj in enumerate(self.objetos):
            pos = obj.obtener_posicion(self.data)
            vel = obj.obtener_velocidad(self.data)
            dadr = dof_addrs[i]

            cinta = self.cintas[obj.segmento]
            fx, fy = cinta.fuerza_transporte(pos, vel)

            self.data.xfrc_applied[obj.body_id, 0] = fx
            self.data.xfrc_applied[obj.body_id, 1] = fy

            if segments[i] == 4 and pos[0] <= OVEN_ENTRY_X:
                self.horno.numero_piezas += 1
                self.objeto_industrial.send_object_back_to_line(model, data, i, dadr, segments)
                continue

            if pos[2] < -0.1 or abs(pos[0]) > 5.0 or abs(pos[1]) > 2.4:
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
                qadr = model.jnt_qposadr[jid]
                data.qpos[qadr:qadr + 3] = [-0.8 + 0.25 * i, -LANE, OBJECT_REST_Z[i]]
                data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
                data.qvel[dadr:dadr + 6] = 0.0
                segments[i] = 0

    def actualizar(self):
        self.aplicar_transporte()
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def mostrar_camaras(self):
        frames = []
        for cam in self.camaras:
            cam.actualizar(self.renderer, self.data)
            frames.append(cam.imagen_con_etiqueta())

        arriba = np.hstack(frames[0:3])
        abajo = np.hstack(frames[3:6])
        mosaico = np.vstack((arriba, abajo))
        cv2.imshow("Camaras de la planta", mosaico)
        return mosaico

    def procesar_teclado(self):
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.corriendo = False
        elif key == ord('p'):
            self.pausa = not self.pausa
        elif key in (ord('1'), ord('2'), ord('3'), ord('4'), ord('5')):
            n = key - ord('1')
            activa = self.cintas[n].conmutar()
            print(f"{self.cintas[n].nombre}: {'ON' if activa else 'OFF'}")
        elif key == ord('r'):
            self.reiniciar_objetos()

    def ejecutar(self):
        self.inicializar()
        ultimo_frame = -1.0

        try:
            while self.corriendo:
                t0 = time.perf_counter()

                if not self.pausa:
                    self.actualizar()

                if ultimo_frame < 0 or (self.data.time - ultimo_frame) >= 1.0 / CAM_FPS:
                    self.mostrar_camaras()
                    ultimo_frame = self.data.time

                self.procesar_teclado()

                espera = self.model.opt.timestep - (time.perf_counter() - t0)
                if espera > 0:
                    time.sleep(espera)
        finally:
            if self.renderer is not None:
                self.renderer.close()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    planta = PlantaIndustrial()
    planta.ejecutar()

