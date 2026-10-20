import mujoco
import numpy as np
from src.config import *

class ObjetoIndustrial:

    def __init__(self, id, forma, segmento, z_reposo):
        self.id = id
        self.nombre = f"obj{id}"
        self.forma = forma
        self.segmento = segmento
        self.z_reposo = z_reposo
        self.body_id = -1
        self.qpos_adr = -1
        self.dof_adr = -1
        self.veces_horno = 0

    def get_ids(self, model):
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.nombre)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{self.nombre}_free")
        self.qpos_adr = model.jnt_qposadr[jid]
        self.dof_adr = model.jnt_dofadr[jid]

    def obtener_posicion(self, data):
        return data.xpos[self.body_id]

    def obtener_velocidad(self, data):
        return data.qvel[self.dof_adr:self.dof_adr + 3]

    def colocar(self, data, x, y, segmento):
        q = self.qpos_adr
        data.qpos[q:q + 3] = [x, y, self.z_reposo]
        data.qpos[q + 3:q + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[self.dof_adr:self.dof_adr + 6] = 0.0
        self.segmento = segmento

    def lock_object_rotations(self, model, data):

        if not LOCK_OBJECT_ROTATION:
            return

        for i in range(5):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
            qadr = model.jnt_qposadr[jid]
            dadr = model.jnt_dofadr[jid]
            data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[dadr + 3:dadr + 6] = 0.0

    def update_segment(self, pos, segmento):
        """Máquina de estados del circuito. La Cinta 5 solo la activa el robot."""
        x, y = float(pos[0]), float(pos[1])

        if segmento == 0 and x >= CORNER_SWITCH:
            return 1
        if segmento == 1 and y >= CORNER_SWITCH:
            return 2
        if segmento == 2 and x <= -CORNER_SWITCH:
            return 3
        if segmento == 3 and y <= -CORNER_SWITCH:
            return 0
        return segmento

    def send_object_back_to_line(self, model, data, i, dadr, segments):
        """Al entrar en el horno, reaparece en Cinta 1 para mantener flujo continuo."""
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
        qadr = model.jnt_qposadr[jid]
        x0 = -1.05 + 0.18 * (i % 3)
        data.qpos[qadr:qadr+3] = [x0, -LANE, OBJECT_REST_Z[i]]
        data.qpos[qadr+3:qadr+7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[dadr:dadr+6] = 0.0
        segments[i] = 0


class Caja(ObjetoIndustrial):

    def __init__(self, id, segmento, z_reposo):
        super().__init__(id, "caja", segmento, z_reposo)



class Cilindro(ObjetoIndustrial):

    def __init__(self, id, segmento, z_reposo):
        super().__init__(id, "cilindro", segmento, z_reposo)

    def centrar_en_cinta(self, data, model, segments):
        if not LOCK_CYLINDERS_TO_CENTERLINE:
            return

        for i in CYLINDER_OBJECT_IDS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
            qadr = model.jnt_qposadr[jid]
            dadr = model.jnt_dofadr[jid]
            x = float(data.qpos[qadr])
            y = float(data.qpos[qadr + 1])
            segmento = segments[i]

            if segmento == 0 and x < TURN_START:
                data.qpos[qadr + 1] = -LANE
                data.qvel[dadr + 1] = 0.0
            elif segmento == 1 and y < TURN_START:
                data.qpos[qadr] = LANE
                data.qvel[dadr] = 0.0
            elif segmento == 2 and x > -TURN_START:
                data.qpos[qadr + 1] = LANE
                data.qvel[dadr + 1] = 0.0
            elif segmento == 3 and y > -TURN_START:
                data.qpos[qadr] = -LANE
                data.qvel[dadr] = 0.0
            elif segmento == 4:
                data.qpos[qadr + 1] = BRANCH_Y
                data.qvel[dadr + 1] = 0.0
