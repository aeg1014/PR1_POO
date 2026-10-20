


class Horno:

    def __init__(self, geom_id=-1, temperatura_inicial=20.0, temp_min=20.0, temp_max=250.0):
        self.geom_id = geom_id
        self.numero_piezas = 0
        self.temperatura = temperatura_inicial
        self.temp_min = temp_min
        self.temp_max = temp_max
        self.encendido = True
        self.pieza_nueva = False

    def registrar_pieza(self, objeto):
        self.numero_piezas += 1
        self.pieza_nueva = True

    def aumentar_temperatura(self, incremento):
        self.temperatura += incremento
        if self.temperatura > self.temp_max:
            self.temperatura = self.temp_max

    def disminuir_temperatura(self, decremento):
        if self.temperatura - decremento < self.temp_min:
            self.temperatura = self.temp_min
        else:
            self.temperatura -= decremento

    def obtener_temperatura(self):
        return self.temperatura

    def actualizar_color(self, model):
        if self.temperatura < 50.0:
            color = [0.1, 0.3, 1.0, 1.0]
        elif self.temperatura < 100.0:
            color = [1.0, 0.6, 0.1, 1.0]
        else:
            color = [1.0, 0.1, 0.1, 1.0]

        model.geom_rgba[self.geom_id] = color

    def procesar_pieza(self, objeto):
        self.registrar_pieza(objeto)
        self.aumentar_temperatura(1)
        self.pieza_nueva = False


