import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import threading
import os
import time

from .logic.lidar import obtener_distancia_angulo, obtener_distancias_rango
from .logic.movement import calcular_rotacion, calcular_movimiento_relativo
from .logic.scene_parser import parsear_escena, leer_camino, guardar_camino
from .logic.planner import planificar
from .logic.execution import PathExecutor
from .logic.relocalization import calcular_qact, distancia_posicion, diferencia_angular_deg


class NavigationNode(Node):
    def __init__(self):
        super().__init__('student_navigation')

        # Suscriptores
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan_raw', self.lidar_callback, 10)

        # Publicador
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Estado interno
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_scan = None
        self._odom_listo = False   # True tras recibir el primer mensaje de odom

        # Memoria para movimientos relativos
        self.target_theta_relativo = None
        self.pose_inicial_relativa = None

        # Escena actual
        self.texto_escena = ""
        self.escena_data = None

        # Comandos del menú
        self.comando_activo = None
        self.parametros_comando = []

        # Modo autónomo
        self._executor = None           # PathExecutor activo
        self._waypoints_plan = []       # waypoints planificados
        self._ruta_camino_txt = None    # ruta del archivo de camino
        self._fase_auto = None          # 'EJECUTANDO' | 'RELOCALIZANDO' | None
        self._reloc_espera = 0          # ticks de espera antes de leer LiDAR final

        # Metricas
        self._t_plan_inicio = 0.0
        self._t_plan_fin = 0.0
        self._t_ejec_inicio = 0.0
        self._t_ejec_fin = 0.0

        # Timer a 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de Navegación Estudiantil Iniciado.")

        # Menú en hilo separado para no bloquear ROS2
        self.hilo_menu = threading.Thread(target=self.menu_interactivo, daemon=True)
        self.hilo_menu.start()

    # =======================================================
    # CALLBACKS DE ROS2
    # =======================================================
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)
        self._odom_listo = True

    def lidar_callback(self, msg):
        self.last_scan = msg

    # =======================================================
    # WRAPPERS PARA LOS ESTUDIANTES
    # =======================================================
    def leer_distancia_en_angulo(self, grados):
        """Retorna la distancia (en metros) de un ángulo específico del Lidar."""
        return obtener_distancia_angulo(self.last_scan, math.radians(grados))

    def leer_distancia_direccion(self, direccion):
        """
        Retorna la distancia en una dirección cardinal:
        'frente', 'atras', 'izquierda', 'derecha'.
        """
        mapa = {
            'frente': 0.0, 'izquierda': 90.0,
            'derecha': 270.0, 'atras': 180.0
        }
        direccion = direccion.lower()
        if direccion in mapa:
            return self.leer_distancia_en_angulo(mapa[direccion])
        self.get_logger().error(f"Dirección '{direccion}' no válida.")
        return float('inf')

    def leer_distancias_en_rango(self, grados_min, grados_max):
        """Retorna una lista con todas las detecciones en un rango visual."""
        return obtener_distancias_rango(self.last_scan, grados_min, grados_max)

    def rotar_relativo(self, grados_relativos, tolerancia=0.05):
        """
        Gira el robot de forma relativa (positivo=izquierda).
        Retorna True si terminó.
        """
        if self.target_theta_relativo is None:
            self.target_theta_relativo = self.current_theta + math.radians(grados_relativos)

        cmd, completado = calcular_rotacion(
            self.current_theta, self.target_theta_relativo, tolerancia=tolerancia)
        self.cmd_pub.publish(cmd)

        if completado:
            self.target_theta_relativo = None
        return completado

    def mover_relativo(self, distancia_x_metros, distancia_y_metros,
                       cono_vision=30, dist_segura=0.3, vel_lineal=0.4):
        """
        Desplazamiento en el frame del robot usando dead-reckoning temporal.
        Retorna 'COMPLETADO' | 'BLOQUEADO' | 'EN_RUTA'.
        """
        if self.pose_inicial_relativa is None:
            self.pose_inicial_relativa = True
            self.tiempo_maniobra = 0.0

        # Dirección de vigilancia anti-choque
        if abs(distancia_x_metros) >= abs(distancia_y_metros):
            if distancia_x_metros >= 0:
                cono = self.leer_distancias_en_rango(-cono_vision, cono_vision)
            else:
                cono = (self.leer_distancias_en_rango(180 - cono_vision, 180) +
                        self.leer_distancias_en_rango(-180, -180 + cono_vision))
        else:
            if distancia_y_metros > 0:
                cono = self.leer_distancias_en_rango(90 - cono_vision, 90 + cono_vision)
            else:
                cono = self.leer_distancias_en_rango(270 - cono_vision, 270 + cono_vision)

        cmd, estado = calcular_movimiento_relativo(
            self.tiempo_maniobra, distancia_x_metros, distancia_y_metros,
            cono, dist_segura=dist_segura, vel_lineal=vel_lineal)
        self.cmd_pub.publish(cmd)

        self.tiempo_maniobra += 0.1

        if estado in ('COMPLETADO', 'BLOQUEADO'):
            self.pose_inicial_relativa = None
            self.tiempo_maniobra = 0.0

        return estado

    def cargar_escena(self, numero_escena):
        """Lee el archivo de la escena indicada y guarda el texto en self.texto_escena."""
        directorio_actual = os.path.dirname(os.path.abspath(__file__))
        ruta_archivo = os.path.join(
            directorio_actual, '..', 'data', f'Escena-Problema{numero_escena}.txt')
        if not os.path.exists(ruta_archivo):
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('proyecto3')
            ruta_archivo = os.path.join(pkg_dir, 'data', f'Escena-Problema{numero_escena}.txt')
        try:
            with open(ruta_archivo, 'r', encoding='utf-8') as archivo:
                self.texto_escena = archivo.read()
            self.escena_data = parsear_escena(self.texto_escena)
            self.get_logger().info(f"Escena {numero_escena} cargada.")
            print(f"\n--- Escena {numero_escena} ---\n{self.texto_escena}\n{'─'*30}")
        except FileNotFoundError:
            self.get_logger().error(f"No se encontró: {ruta_archivo}")
        except Exception as e:
            self.get_logger().error(f"Error al leer la escena: {e}")

    # =======================================================
    # MODO AUTÓNOMO – PLANIFICACIÓN + EJECUCIÓN
    # =======================================================
    def iniciar_autonomo(self, numero_escena):
        """
        Planifica y ejecuta el camino para la escena indicada.
        1. Carga la escena.
        2. Planifica con A*.
        3. Guarda el camino en un .txt.
        4. Ejecuta el camino waypoint a waypoint.
        5. Al terminar: relocaliza y reporta qf, qf-est, qact.
        """
        self.cargar_escena(numero_escena)
        if self.escena_data is None:
            print("Error: no se pudo cargar la escena.")
            return

        print("\n[AUTO] Planificando camino con RRT...")
        self._t_plan_inicio = time.time()
        waypoints = planificar(self.escena_data)
        self._t_plan_fin = time.time()

        if waypoints is None:
            print("[AUTO] ¡Error! No se encontró camino.")
            return

        self._waypoints_plan = waypoints

        # Guardar camino en archivo .txt
        directorio_actual = os.path.dirname(os.path.abspath(__file__))
        self._ruta_camino_txt = os.path.expanduser(
            f'~/camino_escena{numero_escena}.txt')
        guardar_camino(self._ruta_camino_txt, waypoints)
        print(f"[AUTO] Camino guardado en: {self._ruta_camino_txt}")
        print(f"[AUTO] {len(waypoints)} waypoints planificados.")

        # Imprimir camino
        print("[AUTO] Camino:")
        for i, (x, y, th) in enumerate(waypoints):
            print(f"  [{i:2d}] x={x:.3f}  y={y:.3f}  θ={th:.1f}°")

        # Iniciar ejecucion
        self._executor = PathExecutor(waypoints)
        self._fase_auto = 'EJECUTANDO'
        self._t_ejec_inicio = time.time()
        print("\n[AUTO] Iniciando ejecución...\n")

    def _finalizar_autonomo(self):
        """Relocalizacion y reporte al terminar el camino."""
        self.cmd_pub.publish(Twist())  # detener inmediatamente
        self._t_ejec_fin = time.time()
        self._fase_auto = 'RELOCALIZANDO'
        self._reloc_espera = 30  # esperar 3 s parado antes de leer LiDAR

    def _ejecutar_relocalizacion(self):
        self.cmd_pub.publish(Twist())  # mantener detenido
        if self._reloc_espera > 0:
            self._reloc_espera -= 1
            return

        self._fase_auto = None

        escena = self.escena_data
        qf = escena['qf']
        d_frente_teo = escena['dFrente']
        d_derecha_teo = escena['dDerecha']

        # Lectura LiDAR
        d_frente_med = self.leer_distancia_direccion('frente')
        d_derecha_med = self.leer_distancia_direccion('derecha')

        # qf-est (odometría)
        qf_est = (self.current_x, self.current_y, self.current_theta)

        # qact (relocalización LiDAR)
        qact = calcular_qact(
            qf, d_frente_teo, d_derecha_teo,
            d_frente_med, d_derecha_med, self.current_theta)

        # Guardar qf-est y qact en el .txt del camino
        if self._ruta_camino_txt:
            guardar_camino(self._ruta_camino_txt, self._waypoints_plan,
                           qf_est=qf_est, qact=qact)

        # Diferencias
        dist_qf_qfest = distancia_posicion(
            (qf[0], qf[1]), (qf_est[0], qf_est[1]))
        ang_qf_qfest = diferencia_angular_deg(
            qf[2], math.degrees(qf_est[2]))
        dist_qfest_qact = distancia_posicion(
            (qf_est[0], qf_est[1]), (qact[0], qact[1]))
        ang_qfest_qact = diferencia_angular_deg(
            math.degrees(qf_est[2]), qact[2])

        # Metricas del camino
        dist_lineal = 0.0
        suma_angular = 0.0
        wps = self._waypoints_plan
        for i in range(1, len(wps)):
            x0, y0, t0 = wps[i - 1]
            x1, y1, t1 = wps[i]
            dx = x1 - x0
            dy = y1 - y0
            d = math.sqrt(dx * dx + dy * dy)
            # Solo cuenta como traslacion si hay desplazamiento real (> 1 cm)
            if d > 0.01:
                dist_lineal += d
            # Suma absoluta de rotaciones
            dtheta = abs(math.atan2(math.sin(math.radians(t1 - t0)),
                                    math.cos(math.radians(t1 - t0))))
            suma_angular += math.degrees(dtheta)

        t_plan = self._t_plan_fin - self._t_plan_inicio
        t_ejec = self._t_ejec_fin - self._t_ejec_inicio

        sep = '═' * 50
        print(f"\n{sep}")
        print("  RESULTADO FINAL")
        print(sep)
        print(f"  qf      (teórico) : x={qf[0]:.3f}  y={qf[1]:.3f}  θ={qf[2]:.1f}°")
        print(f"  qf-est  (odom)    : x={qf_est[0]:.3f}  y={qf_est[1]:.3f}  "
              f"θ={math.degrees(qf_est[2]):.1f}°")
        print(f"  qact    (LiDAR)   : x={qact[0]:.3f}  y={qact[1]:.3f}  θ={qact[2]:.1f}°")
        print(sep)
        print(f"  LiDAR frente={d_frente_med:.3f} m  (teórico={d_frente_teo:.3f} m)")
        print(f"  LiDAR derecha={d_derecha_med:.3f} m  (teórico={d_derecha_teo:.3f} m)")
        print(sep)
        print(f"  Δ(qf – qf-est):   dist={dist_qf_qfest:.4f} m   Δθ={ang_qf_qfest:.2f}°")
        print(f"  Δ(qf-est – qact): dist={dist_qfest_qact:.4f} m   Δθ={ang_qfest_qact:.2f}°")
        print(sep)
        print("  MÉTRICAS PARA EL INFORME")
        print(sep)
        print(f"  Distancia lineal recorrida : {dist_lineal:.3f} m")
        print(f"  Suma absoluta angular      : {suma_angular:.2f}°")
        print(f"  Tiempo generación RRT      : {t_plan:.3f} s")
        print(f"  Tiempo ejecución simulación: {t_ejec:.3f} s")
        print(f"{sep}\n")

    # =======================================================
    # MENÚ INTERACTIVO
    # =======================================================
    def menu_interactivo(self):
        """Pide input por consola sin interrumpir los sensores de ROS2."""
        while rclpy.ok():
            if self.comando_activo is None and self._fase_auto is None:
                print("\n" + "=" * 40)
                print("       MENÚ DE NAVEGACIÓN")
                print("=" * 40)
                print("1. Leer distancia en un ángulo")
                print("2. Leer distancias en un rango")
                print("3. Rotar grados relativos")
                print("4. Mover relativo a la posición (X, Y)")
                print("5. Cargar Escena de texto")
                print("6. Leer distancia por dirección")
                print("7. ► Modo AUTÓNOMO (planificar + ejecutar escena)")
                print("8. Ejecutar camino desde archivo .txt")
                print("=" * 40)

                try:
                    opcion = input("Elige una opción (1-8): ").strip()

                    if opcion == '1':
                        angulo = float(input("Ángulo (grados): "))
                        self.parametros_comando = [angulo]
                        self.comando_activo = 1

                    elif opcion == '2':
                        ang_min = float(input("Ángulo mínimo: "))
                        ang_max = float(input("Ángulo máximo: "))
                        self.parametros_comando = [ang_min, ang_max]
                        self.comando_activo = 2

                    elif opcion == '3':
                        grados = float(input("Grados (+ izquierda / − derecha): "))
                        self.parametros_comando = [grados]
                        self.comando_activo = 3

                    elif opcion == '4':
                        x = float(input("X en metros (frente/atrás): "))
                        y = float(input("Y en metros (izquierda/derecha): "))
                        self.parametros_comando = [x, y]
                        self.comando_activo = 4

                    elif opcion == '5':
                        numero = int(input("Número de la escena (1-6): "))
                        self.parametros_comando = [numero]
                        self.comando_activo = 5

                    elif opcion == '6':
                        dir_input = input(
                            "Dirección (frente/atras/izquierda/derecha): ").strip().lower()
                        if dir_input in ('frente', 'atras', 'izquierda', 'derecha'):
                            self.parametros_comando = [dir_input]
                            self.comando_activo = 6
                        else:
                            print("Dirección no válida.")

                    elif opcion == '7':
                        numero = int(input("Número de la escena (1-6): "))
                        self.parametros_comando = [numero]
                        self.comando_activo = 7

                    elif opcion == '8':
                        ruta = input("Ruta del archivo .txt: ").strip()
                        self.parametros_comando = [ruta]
                        self.comando_activo = 8

                    else:
                        print("Opción no válida.")

                except ValueError:
                    print("Valor no válido. Intenta de nuevo.")

    # =======================================================
    # BUCLE PRINCIPAL DE CONTROL (10 Hz)
    # =======================================================
    def control_loop(self):
        # ---- Modo autónomo: ejecutar camino solo necesita odom (no LiDAR) ----
        if self._fase_auto == 'EJECUTANDO':
            if not self._odom_listo:   # esperar primer mensaje de odom
                return
            if self._executor is None or self._executor.terminado:
                self.get_logger().info("Camino completado. Iniciando relocalización.")
                self._finalizar_autonomo()
                return

            resultado = self._executor.tick(
                self.current_x, self.current_y, self.current_theta, self)

            if resultado == 'COMPLETADO':
                self.get_logger().info("Camino completado. Iniciando relocalización.")
                self._finalizar_autonomo()
            elif resultado == 'BLOQUEADO':
                self.get_logger().warn("¡Ruta bloqueada! Deteniendo ejecución autónoma.")
                self._fase_auto = None
                self._executor = None
            else:
                if self._executor.idx % 2 == 0:
                    self.get_logger().info(
                        f"Progreso: waypoint {self._executor.progreso}")
            return

        # ---- Relocalización y comandos manuales SÍ necesitan LiDAR ----
        if self.last_scan is None:
            return

        if self._fase_auto == 'RELOCALIZANDO':
            self._ejecutar_relocalizacion()
            return

        # ---- Comandos manuales del menú ----
        if self.comando_activo == 1:
            dist = self.leer_distancia_en_angulo(self.parametros_comando[0])
            self.get_logger().info(
                f"Distancia a {self.parametros_comando[0]}°: {dist:.2f} m")
            self.comando_activo = None

        elif self.comando_activo == 2:
            distancias = self.leer_distancias_en_rango(
                self.parametros_comando[0], self.parametros_comando[1])
            self.get_logger().info(f"Distancias detectadas: {distancias}")
            self.comando_activo = None

        elif self.comando_activo == 3:
            if self.rotar_relativo(self.parametros_comando[0]):
                self.get_logger().info("Rotación completada.")
                self.comando_activo = None

        elif self.comando_activo == 4:
            estado = self.mover_relativo(
                self.parametros_comando[0], self.parametros_comando[1])
            if estado == 'COMPLETADO':
                self.get_logger().info("Desplazamiento completado.")
                self.comando_activo = None
            elif estado == 'BLOQUEADO':
                self.get_logger().warn("¡Obstáculo! Abortando.")
                self.comando_activo = None

        elif self.comando_activo == 5:
            self.cargar_escena(self.parametros_comando[0])
            self.comando_activo = None

        elif self.comando_activo == 6:
            direccion = self.parametros_comando[0]
            dist = self.leer_distancia_direccion(direccion)
            self.get_logger().info(
                f"Distancia hacia {direccion.upper()}: {dist:.2f} m")
            self.comando_activo = None

        elif self.comando_activo == 7:
            numero = self.parametros_comando[0]
            self.iniciar_autonomo(numero)
            self.comando_activo = None

        elif self.comando_activo == 8:
            ruta = self.parametros_comando[0]
            try:
                waypoints = leer_camino(ruta)
                if not waypoints:
                    self.get_logger().error("Archivo vacío o sin waypoints válidos.")
                    self.comando_activo = None
                    return
                self._waypoints_plan = waypoints
                self._ruta_camino_txt = ruta
                self._executor = PathExecutor(waypoints)
                self._fase_auto = 'EJECUTANDO'
                print(f"[AUTO] {len(waypoints)} waypoints cargados. Ejecutando...")
            except Exception as e:
                self.get_logger().error(f"Error cargando camino: {e}")
            self.comando_activo = None


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
