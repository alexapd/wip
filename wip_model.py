# IEE2985 Wheeled Inverted Pendulum

from functools import partial

import numpy as np
from scipy.integrate import solve_ivp  # odeint también?


class Wip:

    def __init__(self):

        # parámetros físicos
        self._r = 2.0  # radio rueda
        self._m = 0.5  # (kg) masa en el péndulo
        self._M = 2.0  # (kg) masa robot
        self._L = 5  # largo del péndulo
        # self._l = 10  # largo del eje de las ruedas

        self._g = 9.81  # aceleración gravedad

        # vectores estado y variables manipuladas
        self._x = np.array([0.0, 0.0, 0.0, 0.0])  # [x, theta, xdot, thetadot]
        self._x_min = np.array([-5.0, -np.pi / 2, -1e6, -1e6])
        self._x_max = np.array([+5.0, +np.pi / 2, +1e6, +1e6])
        self._u = np.array([0.0])  # Fx aplicada a ruedas
        self._u_max = np.array([(self._M + self._m) * 1.5])  # Fx máxima

        # tiempos (en s)
        self._Ts = 0.01  # tiempo de muestreo simulación
        self._t0 = 0.0  # tiempo inicial de intervalo integración
        self._tf = self._t0 + self._Ts  # tiempo final de int. integración
        self._t = 0.0  # tiempo acumulado de simulación

    def SetState(self, x):
        self._x = x  # se impone un estado
        # calculado con ecuacion de movimiento

    def SetActuator(self, u):
        for k in range(len(u)):
            if np.abs(u[k]) > self._u_max[k]:
                u[k] = self._u_max[k] * np.sign(u[k])

        self._u = u

    def _modelo_wip(self, t, x, u):
        """acá se define M, C, G, Fq para luego definir xdot"""

        theta = x[1]
        theta_dot = x[3]
        m = self._m
        M = self._M
        L = self._L
        Fx = u[0]

        Mq = np.array([[M + m, -m * L * np.cos(theta)], [np.cos(theta), -L]])

        b = np.array(
            [[Fx - m * L * theta_dot**2 * np.sin(theta)], [-self._g * np.sin(theta)]]
        )

        qddot = np.linalg.solve(Mq, b)  # queda vector con shape (2,1)

        qdot = x[2:4].reshape(2, 1)

        xdot = np.r_[qdot, qddot]  # queda vector (4,1)

        return xdot.reshape(4)  # esto se le pasa al solve_ivp

    def UpdateState(self):
        """resuelve edo en un pequeño tramo temporal para actualizar estado físico"""
        x0 = self._x
        x = solve_ivp(
            partial(self._modelo_wip, u=self._u), (self._t0, self._tf), x0, method="BDF"
        )

        self._x = ((x.y).T)[-1, :]  # se queda con último estado calculado
        self._t += self._Ts  # se actualiza tiempo de simulación

        # restricción física del chasis robot
        if self._x[1] >= np.pi / 2:
            self._x[1] = np.pi / 2
            if self._x[3] > 0:
                self._x[3] = 0.0
        elif self._x[1] <= -np.pi / 2:
            self._x[1] = -np.pi / 2
            if self._x[3] > 0:
                self._x[3] = 0.0

        # por el momento no hacemos restricciones artificiales:
        # for k in range(len(self._x)):
        #     self._x[k] = max(self._x_min[k], min(self._x[k], self._x_max[k]))

    def GetSensors(self):
        return self._x

    def GetPendulumPosition(self):  # se usa realmente?
        x = self._x[0]
        theta = self._x[1]

        xm = x - self._L * np.sin(theta)
        ym = self._L * np.cos(theta)

        return np.array([xm, ym])


def main():
    wip = Wip()
    print(wip._x)
    wip.UpdateState()
    print(wip._x)

    wip.SetActuator(np.array([10.0]))
    wip.UpdateState()
    print(wip._x)

    wip.UpdateState()
    print(wip._x)

    wip.UpdateState()
    print(wip._x)
    print(wip.GetSensors())


if __name__ == "__main__":
    main()
