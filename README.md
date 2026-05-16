
# Proyecto: Plantilla de Navegación (ROS2)

Este paquete proporciona una base educativa para trabajar con odometría, Lidar y control de movimiento en robots holonómicos o diferenciales.

**Estructura principal**
- `proyecto/` : Código fuente del paquete.
	- `navigation_node.py` : Nodo principal que expone funciones de utilidad para estudiantes (leer Lidar, rotar, mover relativo, cargar escenas, menú interactivo).
	- `logic/lidar.py` : Funciones de ayuda para extraer distancias y rangos del mensaje `LaserScan`.
	- `logic/movement.py` : Algoritmos para calcular velocidades angulares/lineales y perfiles de movimiento relativos.

- `data/` : Archivos de escena (`Escena-Problema1.txt` ... `Escena-Problema6.txt`) usados por el nodo para pruebas y visualización textual.
- `resource/` : Recursos de empaquetado.
- `test/` : Tests básicos y comprobaciones de estilo.

**Características importantes**
- Interfaz de consola no bloqueante: `navigation_node.py` inicia un hilo con un menú interactivo para probar funciones durante la ejecución.
- Wrappers didácticos: Métodos como `leer_distancia_en_angulo`, `leer_distancias_en_rango`, `rotar_relativo`, `mover_relativo` están pensados para que estudiantes llamen fácilmente desde el menú o integren en ejercicios.
- Manejo de escenas: `cargar_escena(numero)` lee el archivo `data/Escena-Problema{numero}.txt` y guarda su contenido en `self.texto_escena`.

**Cómo ejecutar (entorno ROS2)**
1. Fuente del workspace (desde la raíz del workspace):

```bash
colcon build --symlink-install
source install/setup.bash
```

2. Ejecutar el nodo (desde un terminal con ROS2 configurado):

```bash
ros2 run proyecto navigation
```

Nota: El paquete asume tópicos `odom`, `scan_raw` y `cmd_vel` disponibles en el bus ROS2 (simulador Gazebo, robot real o nodo de prueba).

**Editar y extender**
- Para cambiar la lógica del Lidar, revise `proyecto/logic/lidar.py`.
- Para ajustar perfiles de movimiento o tolerancias, edite `proyecto/logic/movement.py`.
- Para añadir nuevas escenas, coloque archivos `Escena-ProblemaX.txt` en la carpeta `data/`.
