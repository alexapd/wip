# wip
wheeled inverted pendulum

Por el momento:
- opción de perturbar la posición del robot a través del teclado
- se implementó control PD (péndulo se estabiliza más o menos bien para pequeñas perturbaciones)

Próximas mejoras:
- ajustar mejor los valores de Kp y Kd, además de agregar un Ki
- arreglar restricciones para pid_out y separarlo de fuerza ingresada por usuario
- resetear cuando el péndulo se haya caído y chocado contra el chasis del robot
- mejorar las gráficas (que no se vea tan pixeleado)
- limitar wip para que no salga de la pantalla (ej: poniendo pared)

Siguientes pasos:
- controlador para el desplazamiento en x (modo auto: usuario elige x_ref)
- implementar otros tipos de controladores
- agregar variables como inercia
