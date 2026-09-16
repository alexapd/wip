# WIP: Log de Actualizaciones
## Wheeled Inverted Pendulum


### 31 de agosto 2026
- opción de perturbar la posición del robot a través del teclado

- implementación control PD

### 10 de septiembre 2026
- separación fuerza del actuador y fx_dist (disturbance por usuario)

- término de simulación una vez que péndulo toca el chasis

- opción de guardar datos en archivo .csv

### 16 de septiembre 2026
- corregir representación wip

- saturar pid_out para evitar windup en caso de que Ki distinto de 0

- implementar circular array para guardar datos

### Próximas mejoras
- ajustar mejor los valores de Kp, Ki, Kd ***

- controlador para el desplazamiento en x (modo auto: usuario elige x_ref)

- implementar otros tipos de controladores (estimar/predecir parámetros)

- agregar variables como inercia y centro de masa y poder cambiarlas

- simular el motor que se ocupará (corriente, etc)

- masa máxima: se la puede el controlador/los motores? inercia distinta. hacer análisis más profundo