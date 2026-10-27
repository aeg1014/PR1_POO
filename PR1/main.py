from src.PlantaIndustrial import PlantaIndustrial


def main():
    planta = PlantaIndustrial(mostrar_visor=True, modo_prueba_ik=True)
    planta.ejecutar()


if __name__ == "__main__":
    main()
