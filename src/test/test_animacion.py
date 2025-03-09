import pygame
from src.animacion.animation import SpotAnimation
from src.utilities.cinematica import FK

def initialize_pygame():
    """
    Inicializa Pygame y crea la ventana principal.
    :return: La superficie de la pantalla y el reloj de Pygame.
    """
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("Spot Animation")
    clock = pygame.time.Clock()
    return screen, clock


def handle_events():
    """
    Maneja los eventos de Pygame (como cerrar la ventana).
    :return: True si el programa debe continuar ejecutándose, False si debe salir.
    """
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
    return True


def calculate_leg_positions(thetas):
    """
    Calcula las posiciones de las patas usando cinemática directa.
    :param thetas: Lista de ángulos para cada pata.
    :return: Datos de las patas (coordenadas de cada segmento).
    """
    legs_data = []
    side = [1,-1,-1,1]
    i = 0
    for theta in thetas:
        leg_position = FK(theta, side[i])  # Ajusta `leg_side` según corresponda
        legs_data.append(leg_position)
        i += 1
    return legs_data


def main():
    """
    Función principal del programa.
    """
    # Inicialización
    screen, clock = initialize_pygame()
    spot_anim = SpotAnimation(screen)

    # Parámetros iniciales
    thetax, thetaz = 0, 0  # Ángulos de rotación de la cámara
    cg_position = [0, 0, 50]  # Posición inicial del centro de gravedad
    thetas = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]  # Ángulos iniciales para las patas

    running = True
    while running:
        # Manejo de eventos
        running = handle_events()
        if not running:
            break

        # Limpia la pantalla
        screen.fill((255, 255, 255))

        # Calcula las posiciones de las patas
        legs_data = calculate_leg_positions(thetas)

        # Dibuja elementos en la pantalla
        spot_anim.draw_axes(thetax, thetaz)
        spot_anim.draw_floor(thetax, thetaz)
        spot_anim.draw_legs(
            legs_data,
            x_spot=0,
            y_spot=0,
            z_spot=0,
            theta_spot=[0, 0, 0, 0, 0, 0],
            thetax=thetax,
            thetaz=thetaz,
        )
        spot_anim.draw_center_of_gravity(cg_position, thetax, thetaz)

        # Actualiza la pantalla
        pygame.display.flip()
        clock.tick(30)

    # Finalización
    pygame.quit()


if __name__ == "__main__":
    main()