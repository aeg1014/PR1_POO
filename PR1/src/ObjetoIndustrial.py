import mujoco

from src.config import (
    BELT_TOP_Z, OBJECT_CLEARANCE_Z, LIMITE_CAIDA_X, LIMITE_CAIDA_Y, LIMITE_CAIDA_Z,
)


class ObjetoIndustrial:
    """Pieza que circula por las cintas. Clase base de Caja y Cilindro."""

    BELT_TOP_Z = BELT_TOP_Z
    OBJECT_CLEARANCE_Z = OBJECT_CLEARANCE_Z

    def __init__(self, numero, color, masa, medio_alto):
        self.numero = numero
        self.nombre = f"obj{numero}"
        self.color = color
        self.masa = masa
        self.medio_alto = medio_alto
        self.forma = "desconocida"
        self.cinta = None

        self.body_id = -1
        self._qpos = -1
        self._dof = -1

    def __str__(self):
        return f"{self.nombre} ({self.forma})"

    @property
    def altura_reposo(self):
        return self.BELT_TOP_Z + self.medio_alto + self.OBJECT_CLEARANCE_Z

    def generar_xml(self):
        """Cada subclase escribe su propia geometría."""
        raise NotImplementedError

    def _xml_cuerpo(self, geom):
        r, g, b = self.color
        return (f'        <body name="{self.nombre}" pos="0 0 {self.altura_reposo}">\n'
                f'            <freejoint name="{self.nombre}_free"/>\n'
                f'            <geom name="{self.nombre}_geom" {geom} mass="{self.masa}" '
                f'rgba="{r} {g} {b} 1"/>\n'
                f'        </body>')

    def conectar(self, model):
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.nombre)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{self.nombre}_free")
        self._qpos = model.jnt_qposadr[jid]
        self._dof = model.jnt_dofadr[jid]

    def posicion(self, data):
        return data.xpos[self.body_id]

    def velocidad(self, data):
        return data.qvel[self._dof:self._dof + 3]

    def colocar(self, data, x, y, cinta):
        """Pone la pieza quieta en (x, y) encima de la cinta indicada."""
        data.qpos[self._qpos:self._qpos + 3] = [x, y, self.altura_reposo]
        data.qpos[self._qpos + 3:self._qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[self._dof:self._dof + 6] = 0.0
        self.cinta = cinta

    def aplicar_fuerza(self, data, fx, fy):
        data.xfrc_applied[self.body_id, 0] = fx
        data.xfrc_applied[self.body_id, 1] = fy

    def actualizar_cinta(self, data):
        """Si la pieza llega al final de su cinta pasa a la siguiente."""
        if self.cinta.ha_terminado(self.posicion(data)):
            self.cinta = self.cinta.siguiente

    def se_ha_caido(self, data):
        x, y, z = self.posicion(data)
        return z < LIMITE_CAIDA_Z or abs(x) > LIMITE_CAIDA_X or abs(y) > LIMITE_CAIDA_Y

    def estabilizar(self, data):
        """Las piezas se trasladan pero no giran."""
        data.qpos[self._qpos + 3:self._qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[self._dof + 3:self._dof + 6] = 0.0


class Caja(ObjetoIndustrial):

    def __init__(self, numero, color, masa, medio_x, medio_y, medio_z):
        super().__init__(numero, color, masa, medio_z)
        self.medidas = (medio_x, medio_y, medio_z)
        if medio_x == medio_y:
            self.forma = "cubo"
        else:
            self.forma = "caja"

    def generar_xml(self):
        x, y, z = self.medidas
        return self._xml_cuerpo(f'type="box" size="{x} {y} {z}"')


class Cilindro(ObjetoIndustrial):

    def __init__(self, numero, color, masa, radio, medio_alto):
        super().__init__(numero, color, masa, medio_alto)
        self.radio = radio
        self.forma = "cilindro"

    def generar_xml(self):
        return self._xml_cuerpo(f'type="cylinder" size="{self.radio} {self.medio_alto}"')

    def estabilizar(self, data):
        super().estabilizar(data)
        # en los tramos rectos el cilindro va justo por el centro de la banda
        if self.cinta.en_tramo_recto(self.posicion(data)):
            eje = self.cinta.eje_transversal()
            data.qpos[self._qpos + eje] = self.cinta.centro[eje]
            data.qvel[self._dof + eje] = 0.0
