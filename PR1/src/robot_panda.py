import numpy as np

from src import robot
from src.config import (
    MODO_PRUEBA_IK, IK_OBJETIVO_POSICION, IK_OBJETIVO_RPY, IK_MARCO_ORIENTACION,
)


class RobotPanda:
    """Envuelve el módulo robot.py (IK ya hecha) para usarlo como un objeto.
    La cinemática inversa no se toca: solo se llaman sus funciones."""

    def __init__(self, config, modo_prueba_ik=MODO_PRUEBA_IK):
        self.config = config
        self.modo_prueba_ik = modo_prueba_ik

        self.objetivo_posicion = np.array(IK_OBJETIVO_POSICION, dtype=float)
        self.objetivo_rpy = np.array(IK_OBJETIVO_RPY, dtype=float)
        self.marco_orientacion = IK_MARCO_ORIENTACION
        self.pinza_abierta = True

        self.ids = None
        self.estado = robot.new_robot_state()

    def conectar(self, model, data):
        self.ids = robot.get_robot_ids(model)
        self.reiniciar(model, data)

    def reiniciar(self, model, data):
        self.estado = robot.new_robot_state()
        robot.initialize_robot(model, data, self.ids, self.estado)
        if self.modo_prueba_ik:
            self.ir_a_objetivo(model)

    # --- objetivos de la IK (modo prueba) ---
    def ir_a_objetivo(self, model):
        robot.set_ik_target_rpy(model, self.ids, self.estado, self.objetivo_posicion,
                                self.objetivo_rpy, orientation_frame=self.marco_orientacion)

    def ir_a_home(self, model):
        robot.set_ik_target_home(model, self.ids, self.estado)

    # --- relación con las piezas ---
    def pieza_agarrada(self):
        return self.estado["grasped_obj"]

    def pieza_reservada(self):
        if self.estado["state"] == "idle":
            return -1
        return self.estado["target_obj"]

    def fuerza_retencion(self, pos, vel, fuerza_maxima):
        return robot.robot_hold_force(pos, vel, self.config, fuerza_maxima)

    # --- control en cada paso ---
    def antes_del_paso(self, model, data, objetos, cintas):
        if self.modo_prueba_ik:
            return
        # robot.py trabaja con listas de números, así que se las preparamos
        body_ids = [obj.body_id for obj in objetos]
        segmentos = [obj.cinta.numero - 1 for obj in objetos]
        alturas = [obj.altura_reposo for obj in objetos]
        robot.update_robot_state_machine(model, data, body_ids, self.ids, self.estado,
                                         segmentos, alturas, self.config)
        for obj, seg in zip(objetos, segmentos):
            obj.cinta = cintas[seg]

    def controlar(self, model, data):
        if self.modo_prueba_ik:
            robot.control_manual_ik(model, data, self.ids, self.estado,
                                    gripper_open=self.pinza_abierta)
        else:
            robot.control_robot_ik(model, data, self.ids, self.estado, self.config)
            robot.follow_tcp_with_grasped_object(model, data, self.ids, self.estado)

    def despues_del_paso(self, model, data):
        if not self.modo_prueba_ik:
            robot.follow_tcp_with_grasped_object(model, data, self.ids, self.estado)

    def texto_estado(self):
        return f"Panda={self.estado['state']} ciclos={self.estado['cycles']}"

    def texto_objetivo(self):
        return robot.ik_target_text(self.estado)
