from math import pi, sin, cos, atan2, sqrt
import numpy as np
from robot.utilities.trama import xyz_rotation_matrix, new_coordinates
from robot.utilities.parametros import *


class Robot:
    phase = pi / 8  # Posición óptima cuando la pierna está completamente levantada
    
    def __init__(self):
        pass

    """
    Función de marcha que genera las posiciones de marcha
    """

    def start_walk_stop(self, track, x_offset, steering_radius, steering_angle, cw, h_amp, v_amp, height, stepl, t,
                        tstep, theta_spot, x_spot, y_spot, z_spot, step_phase):
        
        alpha = np.zeros(4)
        alphav = np.zeros(4)
        theta_spot_updated = theta_spot
        CG = [x_spot[6], y_spot[6], z_spot[6]]

        #Coordenadas del centro de dirección en el marco del punto
        xc, yc = self.calculate_steering_center(steering_radius, steering_angle)
        #Matriz de rotación para la posicion del marco
        Ms = xyz_rotation_matrix(0, 0, theta_spot_updated[2], False)
        #Transformación de las coordenadas del centro de dirección al marco del punto
        xs, ys = self.transform_steering_center(Ms, xc, yc, x_spot, y_spot, z_spot)
        #Matriz de rotación para la posicion del marco
        xn, yn = self.define_foot_positions(track)
        #Calculo de los radios y ángulos de las patas
        radii, an = self.calculate_radii_and_angles(xc, yc, xn, yn)
        #Calculo del ángulo de movimiento
        mangle = self.calculate_movement_angle(radii, h_amp)
        #Calculo del ángulo de rotación y traslación
        dtheta = self.calculate_rotation_angle(mangle, stepl, tstep, cw, step_phase, theta_spot)
        theta_spot_updated[2] += dtheta
        #Actualización de las matrices de rotación
        Ms_updated, Msi_updated, dMs = self.update_rotation_matrices(theta_spot_updated, dtheta)
        foot_center = self.calculate_foot_center(dMs, xs, ys, x_spot, y_spot)
        
        stance_test, stance, t1 = self.initialize_stance_and_time(t, alphav, alpha,step_phase)
        kcomp = 1
        #Comprobación de los pies en el suelo
        
        #Calculo de la compensación        
        x_abs_area, y_abs_area = self.calculate_support_area(x_spot, y_spot)
        x_abs_comp, y_abs_comp = self.calculate_transition_area(t1, stance_test, stance, x_abs_area, y_abs_area, stepl)
        #Calculo de la compensación
        comp, compt, v_amp_t, Msi_comp= self.calculate_compensation(x_abs_comp, y_abs_comp, x_spot, y_spot, CG, x_offset, kcomp, theta_spot_updated, v_amp, step_phase,t1)
        x_framecenter_comp, y_framecenter_comp, z_framecenter_comp = self.calculate_frame_center(foot_center, compt, height)

        x_framecorner, y_framecorner, z_framecorner,x_frame, y_frame = self.calculate_frame_corners(x_framecenter_comp, y_framecenter_comp,
                                                                                     z_framecenter_comp, Ms_updated)

        pos = self.calculate_leg_positions(stance, t1, tstep, x_framecorner, y_framecorner, z_framecorner,
                                           x_spot, y_spot, z_spot, theta_spot_updated, Msi_updated, Ms_updated,
                                           xc, yc, radii, an, mangle, cw, v_amp, alphav, kcomp, x_offset, CG,
                                           alpha, comp, x_frame, y_frame, Msi_comp, foot_center, 
                                           x_framecenter_comp, y_framecenter_comp, z_framecenter_comp)

        return pos

    def calculate_steering_center(self, steering_radius, steering_angle):
        xc = steering_radius * cos(steering_angle)
        yc = steering_radius * sin(steering_angle)
        return xc, yc

    def transform_steering_center(self, Ms, xc, yc, x_spot, y_spot, z_spot):
        s = new_coordinates(Ms, xc, yc, 0, x_spot[0], y_spot[0], z_spot[0])
        xs, ys = s[0], s[1]
        return xs, ys

    def define_foot_positions(self, track):
        xn = [xlf, xrf, xrr, xlr]
        yn = [ylf + track, yrf - track, yrr - track, ylr + track]
        return xn, yn

    def calculate_radii_and_angles(self, xc, yc, xn, yn):
        radii = np.zeros(4)
        an = np.zeros(4)
        for i in range(0, 4):
            radii[i] = sqrt((xc - xn[i]) ** 2 + (yc - yn[i]) ** 2)
            an[i] = atan2(yn[i] - yc, xn[i] - xc)
        return radii, an

    def calculate_movement_angle(self, radii, h_amp):
        maxr = max(radii)
        return h_amp / maxr

    def calculate_rotation_angle(self, mangle, stepl, tstep, cw, step_phase, theta_spot):
        if (step_phase == 'start') or (step_phase == 'stop'):
            dtheta = mangle / (1 - stepl) * tstep / 2 * cw
        else:
            dtheta = mangle / (1 - stepl) * tstep * cw
        return dtheta

    def update_rotation_matrices(self, theta_spot_updated, dtheta):
        Ms_updated = xyz_rotation_matrix(theta_spot_updated[3], theta_spot_updated[4],
                                         theta_spot_updated[2] + theta_spot_updated[5], False)
        Msi_updated = xyz_rotation_matrix(-theta_spot_updated[3], -theta_spot_updated[4],
                                          -(theta_spot_updated[2] + theta_spot_updated[5]), True)
        dMs = xyz_rotation_matrix(0, 0, dtheta, False)
        return Ms_updated, Msi_updated, dMs

    def calculate_foot_center(self, dMs, xs, ys, x_spot, y_spot):
        return new_coordinates(dMs, x_spot[0] - xs, y_spot[0] - ys, 0, xs, ys, 0)

    def initialize_stance_and_time(self, t, alphav,alpha,step_phase):
        stance = [True, True, True, True]
        t1 = t % 1
        for i in range (0,4):
            alphav[i] =0 
            if (t1<=seq[i]):
                 stance[i] = True #La pierna está en el suelo (el valor de la posición absoluta no cambia)
            else:        
                 if (t1<(seq[i] + stepl)):
                     
                     stance[i] = False #La pierna se levanta (el valor de la posición absoluta cambia)
                     alphav[i] = -pi/2+2*pi/stepl*(t1 - seq[i])
                     t2 = seq[i] + stepl
                     if (step_phase == 'start'):
                         #End position alpha 
                         alpha[i] = -seq[i] / (1 - stepl) / 2 + (t2 - seq[i]) / stepl / (1 - stepl) * seq[i]
                     if (step_phase == 'stop'):                          
                         alpha[i] = -1 / 2 + seq[i] / (1 - stepl) / 2 + (t2 - seq[i]) / stepl * (1 - seq[i] / (1 - stepl))
                     if (step_phase == 'walk'):                                                 
                         alpha[i] = -1/2  + ((t2 - seq[i]) / stepl)
                 else:         
                     stance[i] = True #La pierna está en el suelo (el valor de la posición absoluta no cambia)

        """ Cálculo de la compensación """
        stance_test = np.sum(stance) 
        return stance_test,stance, t1

    def calculate_support_area(self, x_spot, y_spot):
        weight = 1.2
        x_abs_area = np.zeros(4)
        y_abs_area = np.zeros(4)

        x_abs_area[0] = ((x_spot[3] + x_spot[5]) * weight + x_spot[4]) / (2 * weight + 1)
        y_abs_area[0] = ((y_spot[3] + y_spot[5]) * weight + y_spot[4]) / (2 * weight + 1)
        x_abs_area[1] = ((x_spot[2] + x_spot[4]) * weight + x_spot[5]) / (2 * weight + 1)
        y_abs_area[1] = ((y_spot[2] + y_spot[4]) * weight + y_spot[5]) / (2 * weight + 1)
        x_abs_area[2] = ((x_spot[3] + x_spot[5]) * weight + x_spot[2]) / (2 * weight + 1)
        y_abs_area[2] = ((y_spot[3] + y_spot[5]) * weight + y_spot[2]) / (2 * weight + 1)
        x_abs_area[3] = ((x_spot[2] + x_spot[4]) * weight + x_spot[3]) / (2 * weight + 1)
        y_abs_area[3] = ((y_spot[2] + y_spot[4]) * weight + y_spot[3]) / (2 * weight + 1)

        return x_abs_area, y_abs_area

    def calculate_transition_area(self, t1,stance_test,stance, x_abs_area, y_abs_area, stepl):
        if  (stance_test == 4): 
             istart = 0
             iend = 0
             #Identificar el inicio y el objetivo de la transición
             tstart = (int(t1/0.25)*0.25)
             tend = tstart+0.25
             if (tend==1):
                 tend = 0
              
             for i in range (0,4):
                 if (tstart == seq[i]):
                     istart = i
                 if (tend  == seq[i]):
                     iend = i
             
             if (t1>(seq[istart] + stepl)):
                 x_abs_comp= x_abs_area[istart]+(x_abs_area[iend]-x_abs_area[istart])*(t1-tstart-stepl)/(0.25-stepl)
                 y_abs_comp= y_abs_area[istart]+(y_abs_area[iend]-y_abs_area[istart])*(t1-tstart-stepl)/(0.25-stepl) 
             else:
                 x_abs_comp = x_abs_area[istart]
                 y_abs_comp = y_abs_area[istart] 
        else:
            for i in range (0,4):
                if (stance[i]==0):
                    x_abs_comp = x_abs_area[i] 
                    y_abs_comp = y_abs_area[i]
        return x_abs_comp, y_abs_comp

    def calculate_compensation(self, x_abs_comp, y_abs_comp, x_spot, y_spot, CG, x_offset, kcomp, theta_spot_updated,v_amp,step_phase,t1):
        Msi_comp = xyz_rotation_matrix(0, 0, -theta_spot_updated[2], True)
        comp = new_coordinates(Msi_comp, x_abs_comp - x_spot[0], y_abs_comp - y_spot[0], 0, 0, 0, 0)
        
        v_amp_t = v_amp
        ts = 0.25
        if (step_phase == 'start'):
            if (t1< ts):
                kcomp = t1/ts
                v_amp_t = 0
        elif (step_phase == 'stop'):  
            if (t1 > (1-ts)):
                kcomp = (1-t1)/ts
                v_amp_t = 0

        Ms_comp = xyz_rotation_matrix(0, 0, theta_spot_updated[2], False)
        compt = new_coordinates(Ms_comp, (comp[0] - CG[0]) * kcomp + x_offset, (comp[1] - CG[1]) * kcomp, 0, 0, 0, 0)

        return comp, compt, v_amp_t, Msi_comp

    def calculate_frame_center(self, foot_center, compt, height):
        x_framecenter_comp = foot_center[0] + compt[0]
        y_framecenter_comp = foot_center[1] + compt[1]
        z_framecenter_comp = height
        return x_framecenter_comp, y_framecenter_comp, z_framecenter_comp

    def calculate_frame_corners(self, x_framecenter_comp, y_framecenter_comp, z_framecenter_comp, Ms_updated):
        x_frame = [xlf, xrf, xrr, xlr]
        y_frame = [ylf, yrf, yrr, ylr]
        z_frame = [0, 0, 0, 0]

        x_framecorner = np.zeros(4)
        y_framecorner = np.zeros(4)
        z_framecorner = np.zeros(4)

        for i in range(0, 4):
            frame_corner = new_coordinates(Ms_updated, x_frame[i], y_frame[i], z_frame[i],
                                           x_framecenter_comp, y_framecenter_comp, z_framecenter_comp)
            x_framecorner[i] = frame_corner[0]
            y_framecorner[i] = frame_corner[1]
            z_framecorner[i] = frame_corner[2]

        return x_framecorner, y_framecorner, z_framecorner, x_frame, y_frame

    def calculate_leg_positions(self, stance, t1, tstep, x_framecorner, y_framecorner, z_framecorner,
                                x_spot, y_spot, z_spot, theta_spot_updated, Msi_updated, Ms_updated,
                                xc, yc, radii, an, mangle, cw, v_amp, alphav, kcomp, x_offset, CG,alpha,comp,
                                x_frame, y_frame, Msi_comp, foot_center, x_framecenter_comp, y_framecenter_comp,
                                z_framecenter_comp):
        xleg = np.zeros(4)
        yleg = np.zeros(4)
        zleg = np.zeros(4)
        xabs = np.zeros(4)
        yabs = np.zeros(4)
        zabs = np.zeros(4)
        xint = np.zeros(4)
        yint = np.zeros(4)
        zint = np.zeros(4)

        for i in range(0, 4):
            if stance[i] == False:
                alphah = an[i] + mangle * alpha[i] * cw
                xleg_target = xc + radii[i] * cos(alphah) - (comp[0] - CG[0]) * kcomp - x_offset - x_frame[i]
                yleg_target = yc + radii[i] * sin(alphah) - (comp[1] - CG[1]) * kcomp - y_frame[i]

                leg_current = new_coordinates(Msi_comp, x_spot[i + 2] - x_framecorner[i],
                                              y_spot[i + 2] - y_framecorner[i], -z_framecorner[i], 0, 0, 0)

                if ((seq[i] + stepl - t1) > tstep):
                    xint[i] = leg_current[0] + (xleg_target - leg_current[0]) * (tstep) / (seq[i] + stepl - t1)
                    yint[i] = leg_current[1] + (yleg_target - leg_current[1]) * (tstep) / (seq[i] + stepl - t1)
                else:
                    xint[i] = xleg_target
                    yint[i] = yleg_target

                zint[i] = leg_current[2] + v_amp * (1 + sin(alphav[i])) / 2

                Msi_body = xyz_rotation_matrix(-theta_spot_updated[3], -theta_spot_updated[4],
                                               -theta_spot_updated[5], True)
                legs = new_coordinates(Msi_body, xint[i], yint[i], zint[i], 0, 0, 0)
                xleg[i] = legs[0]
                yleg[i] = legs[1]
                zleg[i] = legs[2]

                foot_abs = new_coordinates(Ms_updated, xleg[i], yleg[i], zleg[i],
                                           x_framecorner[i], y_framecorner[i], z_framecorner[i])

                xabs[i] = foot_abs[0]
                yabs[i] = foot_abs[1]
                zabs[i] = foot_abs[2]

            else:
                xabs[i] = x_spot[i + 2]
                yabs[i] = y_spot[i + 2]
                zabs[i] = 0

                leg = new_coordinates(Msi_updated, xabs[i] - x_framecorner[i], yabs[i] - y_framecorner[i],
                                      zabs[i] - z_framecorner[i], 0, 0, 0)
                xleg[i] = leg[0]
                yleg[i] = leg[1]
                zleg[i] = leg[2]

        x_spot_updated = [foot_center[0], x_framecenter_comp, xabs[0], xabs[1], xabs[2], xabs[3], x_spot[6], x_spot[7],
                          x_spot[8]]
        y_spot_updated = [foot_center[1], y_framecenter_comp, yabs[0], yabs[1], yabs[2], yabs[3], y_spot[6], y_spot[7],
                          y_spot[8]]
        z_spot_updated = [foot_center[2], z_framecenter_comp, zabs[0], zabs[1], zabs[2], zabs[3], z_spot[6], z_spot[7],
                          z_spot[8]]

        pos = [xleg[0], yleg[0], zleg[0], xleg[1], yleg[1], zleg[1], xleg[2], yleg[2], zleg[2], xleg[3], yleg[3],
               zleg[3], theta_spot_updated, x_spot_updated, y_spot_updated, z_spot_updated]

        return pos

    """
    Función de movimiento desde posiciones de inicio y final conocidas (se utiliza para sentarse, acostarse, etc.)
    """

    def moving(self, t, start_frame_pos, end_frame_pos, pos):
        theta_spot_updated = pos[12]
        x_spot_updated = pos[13]
        y_spot_updated = pos[14]
        z_spot_updated = pos[15]

        frame_pos = self.interpolate_frame_position(start_frame_pos, end_frame_pos, t)

        theta_spot_updated[3] = frame_pos[0]
        theta_spot_updated[4] = frame_pos[1]
        theta_spot_updated[5] = frame_pos[2]

        Mf = xyz_rotation_matrix(frame_pos[0], frame_pos[1], frame_pos[2], False)
        Ms = xyz_rotation_matrix(0, 0, theta_spot_updated[2], False)

        x_frame = [xlf, xrf, xrr, xlr]
        y_frame = [ylf, yrf, yrr, ylr]
        z_frame = [0, 0, 0, 0]

        frame_center_abs = new_coordinates(Ms, frame_pos[3], frame_pos[4], frame_pos[5],
                                           x_spot_updated[0], y_spot_updated[0], z_spot_updated[0])

        x_frame_corner_abs, y_frame_corner_abs, z_frame_corner_abs = self.calculate_absolute_frame_corners(
            Mf, Ms, x_frame, y_frame, z_frame, frame_center_abs)

        xleg, yleg, zleg = self.calculate_relative_leg_positions(theta_spot_updated, x_frame_corner_abs,
                                                                 y_frame_corner_abs, z_frame_corner_abs, x_spot_updated,
                                                                 y_spot_updated, z_spot_updated)

        x_spot_updated[1] = frame_center_abs[0]
        y_spot_updated[1] = frame_center_abs[1]
        z_spot_updated[1] = frame_center_abs[2]

        pos = [xleg[0], yleg[0], zleg[0], xleg[1], yleg[1], zleg[1], xleg[2], yleg[2], zleg[2], xleg[3], yleg[3],
               zleg[3], theta_spot_updated, x_spot_updated, y_spot_updated, z_spot_updated]

        return pos

    def interpolate_frame_position(self, start_frame_pos, end_frame_pos, t):
        frame_pos = np.zeros(6)
        for i in range(0, 6):
            frame_pos[i] = start_frame_pos[i] + (end_frame_pos[i] - start_frame_pos[i]) * t
        return frame_pos

    def calculate_absolute_frame_corners(self, Mf, Ms, x_frame, y_frame, z_frame, frame_center_abs):
        x_frame_corner_abs = np.zeros(4)
        y_frame_corner_abs = np.zeros(4)
        z_frame_corner_abs = np.zeros(4)

        for i in range(0, 4):
            frame_corner = new_coordinates(Mf, x_frame[i], y_frame[i], z_frame[i], 0, 0, 0)
            frame_corner_abs = new_coordinates(Ms, frame_corner[0], frame_corner[1], frame_corner[2],
                                               frame_center_abs[0], frame_center_abs[1], frame_center_abs[2])
            x_frame_corner_abs[i] = frame_corner_abs[0]
            y_frame_corner_abs[i] = frame_corner_abs[1]
            z_frame_corner_abs[i] = frame_corner_abs[2]

        return x_frame_corner_abs, y_frame_corner_abs, z_frame_corner_abs

    def calculate_relative_leg_positions(self, theta_spot_updated, x_frame_corner_abs, y_frame_corner_abs,
                                         z_frame_corner_abs, x_spot_updated, y_spot_updated, z_spot_updated):
        Mi = xyz_rotation_matrix(-theta_spot_updated[3], -theta_spot_updated[4],
                                 -(theta_spot_updated[2] + theta_spot_updated[5]), True)

        xleg = np.zeros(4)
        yleg = np.zeros(4)
        zleg = np.zeros(4)

        for i in range(0, 4):
            leg = new_coordinates(Mi, x_spot_updated[i + 2] - x_frame_corner_abs[i],
                                  y_spot_updated[i + 2] - y_frame_corner_abs[i],
                                  z_spot_updated[i + 2] - z_frame_corner_abs[i], 0, 0, 0)
            xleg[i] = leg[0]
            yleg[i] = leg[1]
            zleg[i] = leg[2]

        return xleg, yleg, zleg