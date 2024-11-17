# Memoria - Práctica ROS2 Turtlesim

ÁLVARO FRAILE CARMENA

---

# Arquitectura del paquete

En esta práctica se usarán diferentes nodos para diferentes funcionalidades, que en su conjunto formarán una arquitectura que se explicará a continuación.

Vamos a tener 2 nodos propios del controlador de turtlesim y el propio nodo de turtlesim. El primero de estos dos nodos será un nodo *keyboard_node*, que se encargará de leer las entradas del teclado y publicarlas sobre un topic llamado *keyboard*. El segundo no será el controlador de turtlesim, es decir, este nodo leerá del topic las entradas del teclado y las procesará, convirtiéndolas en los mensajes adecuados que se publicarán para controlar turtlesim.

![image](https://github.com/user-attachments/assets/e2c1ac4d-d63a-4626-ab52-033d505077e2)

# Lectura del teclado

La lectura del teclado es algo complicada, especialmente la pulsación de multiples teclas simultáneamente. La solución a la que he llegado fue usar las teclas como un toggle, es decir, cuando se pulse una tecla esa tecla estará activa hasta que la volvamos a pulsar. Esto nos permite mover y rotar la tortuga simultáneamente y funcionará correctamente con las funciones de limpiar la traza y activar/desactivar la traza. Donde genera un ligero problema es a la hora de resetear la posición de la tortuga, al activar esta función la tortuga se quedará atrapada en el centro hasta que volvamos a pulsar la tecla.

# Movimiento

Para el movimiento comprobaremos si las teclas alguna de las teclas ‘wasd’ están pulsadas y generaremos el mensaje Twist apropiado.

```python
def process_action(self, msg):
	characters = str(msg.data)
	self.log(10, f"Procesando acción correspondiente al mensaje: {msg.data}")
	
	twist = Twist()
	movement = 0.0
	rotation = 0.0
	
	if "w" in characters:
		movement += self.speed
	if "s" in characters:
		movement += -self.speed
	if "a" in characters:
		rotation += self.rotation_speed
	if "d" in characters:
		rotation += -self.rotation_speed
	...
	self.ts_publisher.publish(twist)
 ![image](https://github.com/user-attachments/assets/b853ebfc-3fac-4e2f-af00-0467bc4f9369)

```

# Acciones

## Limpiar traza

Para limpiar la traza de la tortuga usaremos el servicio *Empty* el cual se encargará de eliminar todas las trazas.

## Ocultar traza

Para ocultar la traza usaremos el servicio *SetPen.* Con este servicio no podemos esconder la traza directamente, pero lo que haremos será alternar el color de la traza entre blanco y el color del fondo según se active.

## Reiniciar posición

Para reiniciar la posición de la tortuga usaremos el servicio *TeleportAbsolute* para teletransportar la tortuga al centro de la ventana con una rotación de 90º.

# Parámetros

Se han creado 2 parámetros que afectan al controlador, *speed* y *log_level* que controlarán la velocidad de la tortuga y el nivel de logging respectivamente.

```python
class TurtlesimController(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')

        self.declare_parameter('speed', 1.0)
        self.declare_parameter('log_level', 20)  
        # (10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL)
				...
				
	 def on_parameter_change(self, params):
        """Callback que se activa cuando cambia algún parámetro."""
        for param in params:
            if param.name == 'speed' and param.type_ == param.Type.DOUBLE:
                try:
                    self.speed = float(param.value)
                    self.log(20, f"Parámetro 'speed' cambiado a {self.speed}")
                except ValueError:
                    self.log(40, f"Valor inválido para 'speed': {param.value}")
                    return SetParametersResult(successful=False)

            elif param.name == 'log_level' and param.type_ == param.Type.INTEGER:
                if 0 <= param.value <= 50:
                    self.log_level = param.value
                    self.log(20, f"Nivel de logging cambiado a {self.log_level}")
                else:
                    self.log(40, f"Nivel de logging fuera de rango: {param.value}")
                    return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)
```

# Launcher

Se ha creado un launcher que lanzará los nodos *controller* y *turtlesim*. **El nodo keyboard requiere ejecutarse de manera independiente**. Se puede lanzar el sistema completo con los siguientes comandos:

```bash
ros2 launch turtlesim_controller launch.py
ros2 run turtlesim_controller keyboard
```

# Video demostrativo

A continuación se encuentra el enlace a un video de la ejecución de todas las funcionalidades:

[https://upm365-my.sharepoint.com/:v:/g/personal/alvaro_fraile_alumnos_upm_es/EYyt2I_xWQNCpJmhXkE_HjIB77-ToAdLQdIXcx98dwH9Aw?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=0MHI6h](https://upm365-my.sharepoint.com/:v:/g/personal/alvaro_fraile_alumnos_upm_es/EYyt2I_xWQNCpJmhXkE_HjIB77-ToAdLQdIXcx98dwH9Aw?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=0MHI6h)
