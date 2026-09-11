# WIP: Log de Actualizaciones
## Wheeled Inverted Pendulum


### 31 de agosto 2026
- opción de perturbar la posición del robot a través del teclado

- implementación control PD

### 10 de septiembre 2026
- separación fuerza del actuador y fx_dist (disturbance por usuario)

- término de simulación una vez que péndulo toca el chasis

- opción de guardar datos en archivo .csv

### Próximas mejoras
- ajustar mejor los valores de Kp, Ki, Kd

- controlador para el desplazamiento en x (modo auto: usuario elige x_ref)

- implementar otros tipos de controladores (estimar/predecir parámetros)

- agregar variables como inercia y centro de masa y poder cambiarlas

- simular el motor que se ocupará (corriente, etc)