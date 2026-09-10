# IEE2985 Wheeled Inverted Pendulum Simulator

import sys

# import time
import numpy as np
import pygame

from wip_model import Wip

# primero, cargar el modelo
wip = Wip()

# variable manipulada
fx_dist = 0.0  # fuerza Fx asociada a disturbance hecha por usuario
pid_out = 0.0  # fuerza Fx asociada a controlador pid ángulo
mv_out = np.array([0.0])  # valor físico al que se traduce

MV_USER_TO_OUT = wip._u_max[0]  ## CAMBIAR?

F_DIST = 20.0  # impulsos/empujones al wip por parte del usuario

# referencias
# x_ref = 0.0
theta_ref = 0.0

# modo de control (automático o manual)
auto = False

# parámetros y variables del controlador PID THETA
Kp_theta = 200.0
Ki_theta = 0.0
Kd_theta = 50.0

error_theta = 0.0
error_theta_old = 0.0
error_theta_old2 = 0.0

# variables gráficas
XMAX = 680
YMAX = 600
screen = None
W2S = 25  # factor de conversión "world to screen"


def update_display():

    # revisar estado actual
    x = wip._x[0]
    theta = wip._x[1]

    # W2S
    floor_y = 450
    x_screen = int(XMAX / 2 + W2S * x)

    # dimensiones físicas
    largo_pendulo = int(W2S * wip._L)

    # representación gráfica robot (se podría modificar)
    body_width = 70
    body_height = 30
    wheel_radius = 15

    screen.fill((0, 0, 0))

    # suelo
    pygame.draw.line(
        screen,
        (255, 255, 255),
        (0, floor_y),
        (XMAX, floor_y),
        2,
    )

    # ruedas
    wheel_y = floor_y - wheel_radius

    left_wheel_x = x_screen - body_width // 2
    right_wheel_x = x_screen + body_width // 2

    pygame.draw.circle(
        screen,
        (100, 100, 100),
        (left_wheel_x, wheel_y),
        wheel_radius,
    )

    pygame.draw.circle(
        screen,
        (100, 100, 100),
        (right_wheel_x, wheel_y),
        wheel_radius,
    )

    # cuerpo de WIP
    body_bottom = wheel_y
    body_top = body_bottom - body_height

    pygame.draw.rect(
        screen,
        (0, 150, 255),
        pygame.Rect(
            x_screen - body_width // 2,
            body_top,
            body_width,
            body_height,
        ),
    )

    # péndulo
    pivot_x = x_screen
    pivot_y = body_top

    mass_x = pivot_x - int(largo_pendulo * np.sin(theta))
    mass_y = pivot_y - int(largo_pendulo * np.cos(theta))

    pygame.draw.line(
        screen,
        (255, 255, 0),
        (pivot_x, pivot_y),
        (mass_x, mass_y),
        5,
    )

    # masa puntual m
    mass_radius = 12

    pygame.draw.circle(
        screen,
        (255, 0, 0),
        (mass_x, mass_y),
        mass_radius,
    )

    # articulación
    pygame.draw.circle(
        screen,
        (200, 200, 200),
        (pivot_x, pivot_y),
        5,
    )

    # info en pantalla
    font = pygame.font.SysFont("lucidasanstypewriter", 18)

    mode = "Auto" if auto else "Manual"

    mode_text = f"Modo: {mode}"
    fx_control_text = f"Fx control = {pid_out:.2f} N"  ##
    dist_text = f"Perturbación = {fx_dist:.2f} N"  ##
    fx_text = f"Fx total = {wip._u[0]:.2f} N"

    screen.blit(
        font.render(mode_text, True, (255, 255, 255)),
        (15, 15),
    )

    screen.blit(
        font.render(fx_control_text, True, (255, 255, 255)),
        (15, 40),
    )

    screen.blit(
        font.render(dist_text, True, (255, 255, 255)),
        (15, 65),
    )

    screen.blit(
        font.render(fx_text, True, (255, 255, 255)),
        (15, 90),
    )

    # finalmente, mostrar todo
    pygame.display.flip()


def u_fun():  # controlador pid ángulo + f_dist, luego retorna mv_out
    """en modo auto: (próximamente) usuario indica x_ref
    en modo manual: usuario puede provocar perturbaciones
    """
    global error_theta, error_theta_old, error_theta_old2
    global pid_out

    theta = wip._x[1]

    error_theta = theta_ref - theta

    # ecuación de PID para salida del controlador
    K0 = Kp_theta + wip._Ts * Ki_theta + Kd_theta / wip._Ts
    K1 = -Kp_theta - 2 * Kd_theta / wip._Ts
    K2 = Kd_theta / wip._Ts

    control_theta = K0 * error_theta + K1 * error_theta_old + K2 * error_theta_old2

    pid_out = pid_out + control_theta

    # CAMBIAR ?
    pid_out = np.clip(
        pid_out,
        -wip._u_max[0],
        wip._u_max[0],
    )

    error_theta_old2 = error_theta_old
    error_theta_old = error_theta

    # CAMBIAR ?
    mv_out[0] = pid_out + fx_dist
    # recordar limitar con wip u max

    return mv_out


def handle_keyboard(wip):

    global auto, fx_dist

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                fx_dist -= F_DIST
            elif event.key == pygame.K_RIGHT:
                fx_dist += F_DIST
            # if event.key == pygame.K_a:
            #     auto = True
            if event.key == pygame.K_m:
                auto = False

        # dejar de apretar tecla significa dejar de causar perturbación
        if (event.type == pygame.KEYUP) and (
            event.key in (pygame.K_LEFT, pygame.K_RIGHT)
        ):
            fx_dist = 0.0


def init_display():
    global screen

    pygame.init()
    screen = pygame.display.set_mode((XMAX, YMAX))
    pygame.display.set_caption("WIP")
    # pygame.key.set_repeat(1, 50)


def main():

    init_display()

    clock = pygame.time.Clock()

    while True:

        handle_keyboard(wip)

        mv_out = u_fun()

        wip.SetActuator(mv_out)
        wip.UpdateState()

        update_display()

        print(
            f"t={wip._t:.2f}, "
            f"x={wip._x[0]:+.3f}, "
            f"theta={wip._x[1]:+.3f}, "
            f"xdot={wip._x[2]:+.3f}, "
            f"thetadot={wip._x[3]:+.3f}, "
            f"Fx={wip._u[0]:+.3f}"
        )

        # hacer un time.wait? o bien un pygame.clock CAMBIAR?
        clock.tick(100)  # concuerda con wip._Ts = 0.01


if __name__ == "__main__":
    main()
