import pygame
from src.utilities.colores import Colors
from src.utilities.coordenadas import Coordinates
from src.utilities.transformaciones import xyz_rotation_matrix, new_coordinates

class SpotAnimation:
    def __init__(self, screen):
        """
        Inicializa la clase de animación.
        :param screen: La superficie de Pygame donde se dibujará la animación.
        """
        self.screen = screen
        self.conv = 0  # Factor de conversión para ajustar la perspectiva

    def draw_axes(self, thetax, thetaz):
        """
        Dibuja los ejes X, Y, Z en la pantalla.
        :param thetax: Ángulo de rotación en el eje X.
        :param thetaz: Ángulo de rotación en el eje Z.
        """
        axes = [Coordinates.X_AXIS, Coordinates.Y_AXIS, Coordinates.Z_AXIS]
        colors = [Colors.RED, Colors.GREEN, Colors.BLUE]

        for axis, color in zip(axes, colors):
            line = self._display_rotate(
                x_spot=0,
                y_spot=0,
                z_spot=0,
                theta_spot=[0, 0, 0, 0, 0, 0],
                thetax=thetax,
                thetaz=thetaz,
                xl=axis["x"],
                yl=axis["y"],
                zl=axis["z"],
            )
            pygame.draw.lines(self.screen, color, False, line, 2)

    def draw_floor(self, thetax, thetaz):
        """
        Dibuja el piso y la cuadrícula en la pantalla.
        :param thetax: Ángulo de rotación en el eje X.
        :param thetaz: Ángulo de rotación en el eje Z.
        """
        # Dibuja el piso principal
        floor_line = self._display_rotate(
            x_spot=0,
            y_spot=0,
            z_spot=0,
            theta_spot=[0, 0, 0, 0, 0, 0],
            thetax=thetax,
            thetaz=thetaz,
            xl=Coordinates.FLOOR["x"],
            yl=Coordinates.FLOOR["y"],
            zl=Coordinates.FLOOR["z"],
        )
        pygame.draw.polygon(self.screen, Colors.GREY, floor_line, 0)

        # Dibuja la cuadrícula del piso
        for i in range(11):
            grid_x = [-500 + i * 100, -500 + i * 100]
            grid_y = [-500, 500]
            grid_z = [0, 0]
            grid_line = self._display_rotate(
                x_spot=0,
                y_spot=0,
                z_spot=0,
                theta_spot=[0, 0, 0, 0, 0, 0],
                thetax=thetax,
                thetaz=thetaz,
                xl=grid_x,
                yl=grid_y,
                zl=grid_z,
            )
            pygame.draw.lines(self.screen, Colors.DARK_GREY, False, grid_line, 1)

            grid_x = [-500, 500]
            grid_y = [-500 + i * 100, -500 + i * 100]
            grid_z = [0, 0]
            grid_line = self._display_rotate(
                x_spot=0,
                y_spot=0,
                z_spot=0,
                theta_spot=[0, 0, 0, 0, 0, 0],
                thetax=thetax,
                thetaz=thetaz,
                xl=grid_x,
                yl=grid_y,
                zl=grid_z,
            )
            pygame.draw.lines(self.screen, Colors.DARK_GREY, False, grid_line, 1)

    def draw_legs(self, legs_data, x_spot, y_spot, z_spot, theta_spot, thetax, thetaz):
        """
        Dibuja las patas del robot.
        :param legs_data: Datos de las patas (coordenadas de cada segmento).
        :param x_spot, y_spot, z_spot: Posición del cuerpo del robot.
        :param theta_spot: Orientación del cuerpo del robot.
        :param thetax, thetaz: Ángulos de rotación de la cámara.
        """
        for leg_data in legs_data:
            x_leg, y_leg, z_leg = leg_data["x"], leg_data["y"], leg_data["z"]
            line = self._display_rotate(
                x_spot=x_spot,
                y_spot=y_spot,
                z_spot=z_spot,
                theta_spot=theta_spot,
                thetax=thetax,
                thetaz=thetaz,
                xl=x_leg,
                yl=y_leg,
                zl=z_leg,
            )
            pygame.draw.lines(self.screen, Colors.RED, False, line, 4)

    def draw_center_of_gravity(self, cg_position, thetax, thetaz):
        """
        Dibuja el centro de gravedad del robot.
        :param cg_position: Posición del centro de gravedad [x, y, z].
        :param thetax, thetaz: Ángulos de rotación de la cámara.
        """
        line_cg = self._display_rotate(
            x_spot=0,
            y_spot=0,
            z_spot=0,
            theta_spot=[0, 0, 0, 0, 0, 0],
            thetax=thetax,
            thetaz=thetaz,
            xl=[cg_position[0], cg_position[0]],
            yl=[cg_position[1], cg_position[1]],
            zl=[0, cg_position[2]],
        )
        pygame.draw.lines(self.screen, Colors.BLACK, False, line_cg, 1)
        pygame.draw.circle(self.screen, Colors.DARK_GREY, line_cg[1], 10)

    def _display_rotate(self, x_spot, y_spot, z_spot, theta_spot, thetax, thetaz, xl, yl, zl):
        """
        Transforma las coordenadas 3D a 2D para su visualización.
        :param x_spot, y_spot, z_spot: Posición del cuerpo del robot.
        :param theta_spot: Orientación del cuerpo del robot.
        :param thetax, thetaz: Ángulos de rotación de la cámara.
        :param xl, yl, zl: Coordenadas de entrada.
        :return: Lista de puntos 2D para dibujar.
        """
        line = []
        Ma = xyz_rotation_matrix(theta_spot[3], theta_spot[4], theta_spot[2] + theta_spot[5], False)
        Mb = xyz_rotation_matrix(theta_spot[0], theta_spot[1], 0, False)
        M1 = xyz_rotation_matrix(thetax, 0, thetaz, True)

        for i in range(len(xl)):
            out0 = new_coordinates(Ma, xl[i], yl[i], zl[i], x_spot, y_spot, z_spot)
            out = new_coordinates(Mb, out0[0], out0[1], out0[2], 0, 0, 0)
            disp = new_coordinates(M1, out[0], out[1], out[2], 0, 0, 0)

            yd = disp[1]
            xd = disp[0] / 2 ** (yd * self.conv / 2000)
            zd = disp[2] / 2 ** (yd * self.conv / 2000)

            line.append([int(300 + xd), int(300 - zd)])

        return line