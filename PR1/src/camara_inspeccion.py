import cv2



class CamaraInspeccion:

    def __init__(self, nombre, titulo, cinta=None):
        self.nombre = nombre
        self.titulo = titulo
        self.cinta = cinta
        self.objeto_detectado = False
        self.forma_detectada = None
        self.imagen = None

    def capturar(self, renderer, data):
        renderer.update_scene(data, camera=self.nombre)
        rgb = renderer.render()
        self.imagen = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return self.imagen

    def detectar_forma(self, imagen):
        gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
        gris = cv2.GaussianBlur(gris, (5, 5), 0)
        _, mascara = cv2.threshold(gris, 100, 255, cv2.THRESH_BINARY)

        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contornos) == 0:
            return None

        c = max(contornos, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 500:
            return None

        perimetro = cv2.arcLength(c, True)
        aprox = cv2.approxPolyDP(c, 0.04 * perimetro, True)


        return None

    def actualizar(self, renderer, data):
        imagen = self.capturar(renderer, data)

        self.forma_detectada = self.detectar_forma(imagen)
        self.objeto_detectado = self.forma_detectada is not None

        if self.objeto_detectado and self.cinta is not None:
            self.cinta.detener()

        return imagen

    def imagen_con_etiqueta(self, estado=None):
        img = self.imagen.copy()
        cv2.rectangle(img, (0, 0), (img.shape[1], 36), (20, 20, 20), -1)
        cv2.putText(img, self.titulo, (12, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (245, 245, 245), 2, cv2.LINE_AA)
        if estado:
            cv2.putText(img, estado, (12, img.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (245, 245, 245), 1, cv2.LINE_AA)
        return img

if __name__ == "__main__":
    camara = CamaraInspeccion(nombre="camara_inspeccion", titulo="Camara de Inspección")
    camara.capturar()
