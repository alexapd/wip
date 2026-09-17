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
- controlador para el desplazamiento en x (modo auto: usuario elige x_ref)

- posibilidad de ajustar valores Ki, Kp, Kd con widgets (parte didáctica)

- ajustar con variables nominales de modelo

- testear controladores con programa externo y ver valores adecuados

- cambiar tipo de controlador (estimar/predecir parámetros)