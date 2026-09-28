from wip_model import Wip
import numpy as np
from scipy.optimize import differential_evolution

# import time
# import sys

N = 1000  # número de frames de cada simulación

theta_ref = np.deg2rad(0.0)
theta_inicial = np.deg2rad(2.0)

# rango de búsqueda automática, CAMBIAR SI ES NECESARIO
bounds = [(0.0, 20.0), (0.0, 20.0), (0.0, 20.0)]  # Kp, Ki, Kd

# para guardar combinaciones K's y su respectivo costo
historial = []


# repetir x veces según algoritmo de búsqueda
def simulacion(Kp, Ki, Kd, guardar=False):

    wip = Wip()
    wip._x[1] = theta_inicial

    Ts = wip._Ts
    costo = 0

    error_integral = 0.0

    theta_actual = wip._x[1]
    error_old = theta_ref - theta_actual

    datos = []  # para ir guardando t, theta_ref, theta_actual, error_theta, pid_out

    for k in range(N):

        # actual
        t = k * Ts
        theta_actual = wip._x[1]

        # cálculo de PID CAMBIAR SI ES NECESARIO
        error_theta = theta_ref - theta_actual

        error_integral += error_theta * Ts

        error_derivada = (error_theta - error_old) / Ts

        pid_out = Kp * error_theta + Ki * error_integral + Kd * error_derivada

        error_old = error_theta

        # aplicar controlador y seguir simulando
        wip.SetActuator(np.array([pid_out]))
        wip.UpdateState()

        # según ITAE
        costo += (
            t * abs(error_theta) * Ts
        )  # se multiplica por Ts para cancelar efecto de frecuencia

        if guardar:
            datos.append([t, theta_ref, theta_actual, error_theta, pid_out])

    if guardar:
        return costo, np.array(datos)

    return costo


# inicio = time.perf_counter()
# costo = simulacion(200, 0, 50)
# fin = time.perf_counter()
# print("una simulación tarda", fin - inicio, "s")
# sys.exit()


# función minimizada por algoritmo de búsqueda
def funcion_costo(K):
    Kp, Ki, Kd = K

    costo = simulacion(Kp, Ki, Kd)

    historial.append([Kp, Ki, Kd, costo])

    return costo


# aplicar algoritmo de differential evolution
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.differential_evolution.html
resultado = differential_evolution(
    funcion_costo, bounds, popsize=5, maxiter=20, polish=False, disp=True, seed=1
)


print("Mejores ganancias encontradas:")
print("Kp =", resultado.x[0])
print("Ki =", resultado.x[1])
print("Kd =", resultado.x[2])
print("ITAE =", resultado.fun)


# se ordenan las simulaciones por costo en orden creciente
historial = np.array(historial)
historial = historial[np.argsort(historial[:, 3])]

mejores = historial[:2]
peores = historial[-2:]

# se guardan los dos mejores y dos peores casos
casos = [
    ("mejor_1", mejores[0]),
    ("mejor_2", mejores[1]),
    ("peor_1", peores[-1]),
    ("peor_2", peores[-2]),
]

# se guardan datos en archivo para ser graficados
for nombre, caso in casos:

    Kp, Ki, Kd, costo = caso

    _, datos = simulacion(Kp, Ki, Kd, guardar=True)

    np.savetxt(
        f"{nombre}.csv",
        datos,
        delimiter=",",
        header="t, theta_ref, theta, error, pid_out",
        comments="",
    )

# próximas mejoras:
# incluir pid_out en optimización controlador
# --> se relaciona a saturación del motor
# ajustar configuraciones de differential evolution
