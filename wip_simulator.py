# IEE2985 Wheeled Inverted Pendulum Simulator

import sys

# import time
import numpy as np
import pygame
from pathlib import Path

from wip_model import Wip

# primero, cargar el modelo
wip = Wip()

# variable manipulada
mv_out = np.array([0.0])  # valor físico al que se traduce

fx_dist = 0.0  # fuerza Fx asociada a disturbance hecha por usuario
F_DIST = 20.0  # impulsos/empujones al wip por parte del usuario

# controladores
pid_out_theta = 0.0  # fuerza Fx asociada a controlador pid ángulo
pid_out_x = 0.0  # representa referencia angular necesaria

# referencias
x_ref = 0.0
theta_ref = 0.0

# modo de control (automático o manual)
auto = False

# parámetros y variables del controlador PID THETA
Kp_theta = 200.0
Ki_theta = 0.0
Kd_theta = 50.0

Kp_x = 0.05
Ki_x = 0.0
Kd_x = 0.0

error_theta = 0.0
error_theta_old = 0.0
error_theta_old2 = 0.0

error_x = 0.0
error_x_old = 0.0
error_x_old2 = 0.0

# variables gráficas
XMAX = 680
YMAX = 600
screen = None
W2S = 25  # factor de conversión "world to screen"

# variables para guardar datos simulación
data_full = False
k_data = 0
k_data_samples = 10000
record_size = 8
data_sim = np.zeros((k_data_samples, record_size))  # datos de simulación por guardar


def store_data():
    """guarda datos del frame en el circular array"""
    global k_data, data_full

    # guardar los datos [t, x, theta, xdot, thetadot, fx_actuador, fx_dist, fx_total]
    data_sim[k_data, :] = np.r_[
        wip._t,
        wip._x[0],
        wip._x[1],
        wip._x[2],
        wip._x[3],
        wip._u[0],
        wip._dist[0],
        wip._u[0] + wip._dist[0],
    ]
    k_data += 1

    if k_data == k_data_samples:
        k_data = 0
        data_full = True


def arrange_data(data_sim):
    """ordena los datos del circular array de antiguo a nuevo"""

    if not data_full:
        return data_sim[:k_data, :]

    aux1 = data_sim[:k_data, :]  # los primeros valores
    aux2 = data_sim[-(k_data_samples - k_data) :, :]  # el resto
    data_sim = np.r_[aux2, aux1]  # poner verticalmente

    return data_sim


def guardar_archivo_data(data_sim):
    data_folder = Path("datos")
    data_folder.mkdir(exist_ok=True)

    i = 1

    while True:
        filename = data_folder / f"wip_simulation_data_{i}.csv"

        if not filename.exists():
            break

        i += 1

    with open(filename, "w", encoding="utf-8") as file:
        file.write(f"# Kp_theta={Kp_theta}\n")
        file.write(f"# Ki_theta={Ki_theta}\n")
        file.write(f"# Kd_theta={Kd_theta}\n")

        file.write(f"# M={wip._M}\n")
        file.write(f"# m={wip._m}\n")
        file.write(f"# L={wip._L}\n")
        file.write(f"# r={wip._r}\n")

        file.write("t,x,theta,xdot,thetadot,F_act,F_dist,Fx_total\n")

        data_ordenada = arrange_data(data_sim)

        np.savetxt(
            file,
            data_ordenada,
            delimiter=",",
        )

    print(f"Datos guardados en: {filename}")


def termino_simulacion():
    pygame.quit()

    resp = input("¿Quieres guardar los datos de la simulación? [s/n]: ")

    if resp.lower() == "s":
        guardar_archivo_data(data_sim)

    sys.exit()


def update_display():

    # revisar estado actual
    x = wip._x[0]
    theta = wip._x[1]

    # W2S
    floor_y = 450
    x_screen = int(XMAX / 2 + W2S * x)

    # distancia de eje de ruedas a centro de masa
    largo_pendulo = int(W2S * wip._L)

    # representación gráfica robot
    wheel_radius = 30

    screen.fill((0, 0, 0))

    # suelo
    pygame.draw.line(
        screen,
        (255, 255, 255),
        (0, floor_y),
        (XMAX, floor_y),
        2,
    )

    # ruedas del wip
    wheel_y = floor_y - wheel_radius
    wheel_x = x_screen

    pygame.draw.circle(
        screen,
        (0, 0, 255),
        (wheel_x, wheel_y),
        wheel_radius,
    )

    # péndulo
    pivot_x = x_screen
    pivot_y = wheel_y

    mass_x = pivot_x - int(largo_pendulo * np.sin(theta))
    mass_y = pivot_y - int(largo_pendulo * np.cos(theta))

    pygame.draw.line(
        screen,
        (255, 255, 0),
        (pivot_x, pivot_y),
        (mass_x, mass_y),
        5,
    )

    # masa puntual m que representa centro de masa
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
    fx_control_text = f"Fx control = {wip._u[0]:.2f} N"  ##
    dist_text = f"Perturbación = {wip._dist[0]:.2f} N"  ##
    fx_text = f"Fx total = {wip._u[0] + wip._dist[0]:.2f} N"

    x_ref_text = f"x_ref = {x_ref:.2f} m"
    x_text = f"x = {wip._x[0]:.2f} m"

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

    if auto:
        screen.blit(
            font.render(x_text, True, (255, 255, 255)),
            (300, 15),
        )
        screen.blit(
            font.render(x_ref_text, True, (255, 255, 255)),
            (300, 40),
        )

    # finalmente, mostrar todo
    pygame.display.flip()


def control_posicion():
    """calcula theta_ref para que controlador theta haga que robot se desplace a x_ref"""

    global error_x, error_x_old, error_x_old2
    global pid_out_x

    x = wip._x[0]

    error_x = x_ref - x

    K0 = Kp_x + wip._Ts * Ki_x + Kd_x / wip._Ts
    K1 = -Kp_x - 2 * Kd_x / wip._Ts
    K2 = Kd_x / wip._Ts

    control_x = K0 * error_x + K1 * error_x_old + K2 * error_x_old2

    pid_out_x = pid_out_x + control_x

    error_x_old2 = error_x_old
    error_x_old = error_x

    theta_ref_max = wip._theta_ref_max

    pid_out_x = np.clip(pid_out_x, -theta_ref_max, theta_ref_max)

    return pid_out_x


def u_fun():  # controlador pid ángulo luego retorna mv_out
    """controlador pid de ángulo a partir de theta_ref
    modo manual: theta_ref = 0
    modo auto: theta_ref fijada por controlador de posición"""

    global error_theta, error_theta_old, error_theta_old2
    global pid_out_theta

    theta = wip._x[1]

    error_theta = theta_ref - theta

    # ecuación de PID para salida del controlador
    K0 = Kp_theta + wip._Ts * Ki_theta + Kd_theta / wip._Ts
    K1 = -Kp_theta - 2 * Kd_theta / wip._Ts
    K2 = Kd_theta / wip._Ts

    control_theta = K0 * error_theta + K1 * error_theta_old + K2 * error_theta_old2

    pid_out_theta = pid_out_theta + control_theta

    error_theta_old2 = error_theta_old
    error_theta_old = error_theta

    # pid_out_theta = 0  # añadir esta línea para probar física del modelo

    # para evitar windup en caso de que Ki distinto de 0
    pid_out_theta = np.clip(
        pid_out_theta,
        -wip._u_max[0],
        wip._u_max[0],
    )

    mv_out[0] = pid_out_theta

    return mv_out


def handle_keyboard():

    global auto, fx_dist

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            termino_simulacion()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                fx_dist -= F_DIST
            elif event.key == pygame.K_RIGHT:
                fx_dist += F_DIST
            elif event.key == pygame.K_a:
                auto = True
            elif event.key == pygame.K_m:
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


def escoger_parametros():
    global x_ref
    global Kp_x, Ki_x, Kd_x

    print("\n --- Control de posición ---")

    print("recomendación: escoger x_ref entre -5 m y 5 m")
    x_ref = float(input("x_ref [m]: "))

    Kp_x = float(input("Kp_x: "))
    Ki_x = float(input("Ki_x: "))
    Kd_x = float(input("Kd_x: "))


def main():
    global theta_ref

    escoger_parametros()
    init_display()

    clock = pygame.time.Clock()

    while True:

        handle_keyboard()

        if auto:
            theta_ref = control_posicion()
        else:
            theta_ref = 0.0

        mv_out = u_fun()

        wip.SetActuator(mv_out)  # con saturación de actuador incluida
        wip.SetDisturbance(np.array([fx_dist]))  # fuerza externa, no se satura

        colision = wip.UpdateState()

        # guardar los datos [t, x, theta, xdot, thetadot, fx_actuador, fx_dist, fx_total]
        store_data()

        update_display()

        if colision:
            print("\nEl péndulo chocó con el suelo.")
            termino_simulacion()

        print(
            f"t={wip._t:.2f}, "
            f"x={wip._x[0]:+.3f}, "
            f"theta={wip._x[1]:+.3f}, "
            f"xdot={wip._x[2]:+.3f}, "
            f"thetadot={wip._x[3]:+.3f}, "
            f"F_act={wip._u[0]:+.3f}, "
            f"F_dist={wip._dist[0]:+.3f}, "
            f"Fx_total={wip._u[0] + wip._dist[0]:+.3f}"
        )

        clock.tick(100)  # concuerda con wip._Ts = 0.01


if __name__ == "__main__":
    main()
