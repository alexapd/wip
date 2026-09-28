import numpy as np
import matplotlib.pyplot as plt

archivos = [
    ("mejor_1.csv", "Mejor 1"),
    ("mejor_2.csv", "Mejor 2"),
    ("peor_1.csv", "Peor 1"),
    ("peor_2.csv", "Peor 2"),
]

# gráficos theta vs tiempo para ver evolución error_theta
plt.figure()

for archivo, nombre in archivos:

    data = np.loadtxt(archivo, delimiter=",", skiprows=1)

    t = data[:, 0]
    theta_ref = data[:, 1]
    theta = data[:, 2]

    plt.plot(t, np.rad2deg(theta), label=nombre)

# referencia
plt.plot(t, np.rad2deg(theta_ref), "--", label="Referencia")

plt.xlabel("Tiempo [s]")
plt.ylabel("Ángulo θ [°]")
plt.title("Comparación de controladores PID")
plt.grid()
plt.legend()

plt.show()

# gráficos pid_out para ver exigencia que pide controlador
plt.figure()

for archivo, nombre in archivos:

    data = np.loadtxt(archivo, delimiter=",", skiprows=1)

    t = data[:, 0]
    pid_out = data[:, 4]

    plt.plot(t, pid_out, label=nombre)

plt.xlabel("Tiempo [s]")
plt.ylabel("PID output")
plt.title("Acción de control")
plt.grid()
plt.legend()

plt.show()
