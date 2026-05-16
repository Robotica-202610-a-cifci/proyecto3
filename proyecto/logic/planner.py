"""
Planificador de caminos geometricos usando RRT (Rapidly-exploring Random Tree).

El muestreo se aplica directamente en el espacio continuo de configuraciones
sin cuadricula, sin restriccion en la direccion de desplazamiento.

El robot se modela como un circulo de radio ROBOT_RADIO metros para
el calculo de colisiones con obstaculos rectangulares convexos.
"""
import math
import random

ROBOT_RADIO = 0.28      # metros – igual que en proyecto 2
STEP_SIZE = 0.30        # metros por expansion
GOAL_RADIUS = 0.25      # metros – umbral para considerar el objetivo alcanzado
GOAL_BIAS = 0.10        # probabilidad de muestrear directamente el objetivo
MAX_ITER = 8000         # iteraciones maximas
MAX_SEG_M = 1.0         # maximo metros por segmento recto en el camino final


# ---------------------------------------------------------------------------
# Geometria: colision segmento–rectangulo inflado
# ---------------------------------------------------------------------------

def _segmento_libre(p1, p2, obstaculos, robot_r, ancho, alto):
    """
    Verifica que el segmento p1-p2 no colisione con ningun obstaculo
    (rectangulares, inflados por robot_r) ni con las paredes del escenario.
    Usa muestreo denso a lo largo del segmento.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dist = math.sqrt(dx * dx + dy * dy)
    pasos = max(int(dist / (robot_r * 0.5)) + 1, 4)

    for i in range(pasos + 1):
        t = i / pasos
        x = p1[0] + t * dx
        y = p1[1] + t * dy

        # Paredes
        if x < robot_r or x > ancho - robot_r:
            return False
        if y < robot_r or y > alto - robot_r:
            return False

        # Obstaculos inflados
        for obs in obstaculos:
            x1, y1 = obs['pto1']
            x2, y2 = obs['pto2']
            xmin = min(x1, x2) - robot_r
            xmax = max(x1, x2) + robot_r
            ymin = min(y1, y2) - robot_r
            ymax = max(y1, y2) + robot_r
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return False

    return True


def _punto_libre(x, y, obstaculos, robot_r, ancho, alto):
    """Verifica que un punto no este en colision."""
    if x < robot_r or x > ancho - robot_r:
        return False
    if y < robot_r or y > alto - robot_r:
        return False
    for obs in obstaculos:
        x1, y1 = obs['pto1']
        x2, y2 = obs['pto2']
        xmin = min(x1, x2) - robot_r
        xmax = max(x1, x2) + robot_r
        ymin = min(y1, y2) - robot_r
        ymax = max(y1, y2) + robot_r
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return False
    return True


# ---------------------------------------------------------------------------
# RRT
# ---------------------------------------------------------------------------

def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _nearest(nodes, sample):
    best_idx = 0
    best_d = _dist(nodes[0], sample)
    for i in range(1, len(nodes)):
        d = _dist(nodes[i], sample)
        if d < best_d:
            best_d = d
            best_idx = i
    return best_idx


def _steer(origen, destino, step):
    d = _dist(origen, destino)
    if d <= step:
        return destino
    t = step / d
    return (origen[0] + t * (destino[0] - origen[0]),
            origen[1] + t * (destino[1] - origen[1]))


def _rrt(q0_xy, qf_xy, obstaculos, ancho, alto):
    """
    Ejecuta RRT desde q0_xy hasta qf_xy.
    Retorna lista de (x, y) o None si no encuentra solucion.
    """
    nodes = [q0_xy]
    parents = [-1]

    for _ in range(MAX_ITER):
        # Muestreo con sesgo hacia el objetivo
        if random.random() < GOAL_BIAS:
            sample = qf_xy
        else:
            sample = (random.uniform(0, ancho), random.uniform(0, alto))

        # Nodo mas cercano
        near_idx = _nearest(nodes, sample)
        near = nodes[near_idx]

        # Expansion
        new_node = _steer(near, sample, STEP_SIZE)

        if not _punto_libre(new_node[0], new_node[1], obstaculos, ROBOT_RADIO, ancho, alto):
            continue

        if not _segmento_libre(near, new_node, obstaculos, ROBOT_RADIO, ancho, alto):
            continue

        nodes.append(new_node)
        parents.append(near_idx)

        # Verificar si llegamos al objetivo
        if _dist(new_node, qf_xy) < GOAL_RADIUS:
            # Intentar conectar directamente al objetivo exacto
            if _segmento_libre(new_node, qf_xy, obstaculos, ROBOT_RADIO, ancho, alto):
                nodes.append(qf_xy)
                parents.append(len(nodes) - 2)

            # Reconstruir camino
            path = []
            idx = len(nodes) - 1
            while idx != -1:
                path.append(nodes[idx])
                idx = parents[idx]
            path.reverse()
            return path

    return None


# ---------------------------------------------------------------------------
# Post-procesamiento: suavizado y segmentacion
# ---------------------------------------------------------------------------

def _suavizar(path_xy, obstaculos, ancho, alto):
    """Elimina nodos redundantes usando line-of-sight shortcuts."""
    if len(path_xy) <= 2:
        return path_xy

    resultado = [path_xy[0]]
    i = 0
    while i < len(path_xy) - 1:
        j = len(path_xy) - 1
        while j > i + 1:
            if _segmento_libre(path_xy[i], path_xy[j], obstaculos, ROBOT_RADIO, ancho, alto):
                break
            j -= 1
        resultado.append(path_xy[j])
        i = j
    return resultado


def _partir_segmentos_largos(path_xy):
    """Parte segmentos mas largos que MAX_SEG_M en trozos mas cortos."""
    resultado = [path_xy[0]]
    for i in range(1, len(path_xy)):
        p0 = resultado[-1]
        p1 = path_xy[i]
        dist = _dist(p0, p1)
        if dist > MAX_SEG_M:
            partes = math.ceil(dist / MAX_SEG_M)
            for k in range(1, partes):
                t = k / partes
                mid = (p0[0] + t * (p1[0] - p0[0]),
                       p0[1] + t * (p1[1] - p0[1]))
                resultado.append(mid)
        resultado.append(p1)
    return resultado


# ---------------------------------------------------------------------------
# Conversion de ruta XY a waypoints (x, y, theta) con rotacion + traslacion
# ---------------------------------------------------------------------------

def _angulo_entre(p1, p2):
    return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))


def _camino_a_waypoints(path_xy, q0, qf):
    """
    Convierte lista de (x, y) en waypoints (x, y, theta) que alternan:
      [rotacion, traslacion, rotacion, traslacion, ..., rotacion_final]
    Compatible con PathExecutor del proyecto 2.
    """
    waypoints = []
    waypoints.append((q0[0], q0[1], q0[2]))

    for i in range(1, len(path_xy)):
        x_prev, y_prev = path_xy[i - 1]
        x_curr, y_curr = path_xy[i]

        theta_nueva = _angulo_entre((x_prev, y_prev), (x_curr, y_curr))

        # Configuracion de rotacion (misma posicion, nuevo angulo)
        waypoints.append((waypoints[-1][0], waypoints[-1][1], theta_nueva))

        # Configuracion de traslacion (nueva posicion, mismo angulo)
        waypoints.append((x_curr, y_curr, theta_nueva))

    # Ajustar ultimo punto a las coordenadas exactas de qf
    waypoints[-1] = (qf[0], qf[1], waypoints[-1][2])

    # Rotacion final a la orientacion de qf
    waypoints.append((qf[0], qf[1], qf[2]))

    return waypoints


# ---------------------------------------------------------------------------
# API publica
# ---------------------------------------------------------------------------

def planificar(escena):
    """
    Planifica el camino de q0 a qf usando RRT en espacio continuo.

    Retorna lista de (x, y, theta_grados) o None si no encuentra solucion.
    """
    ancho = escena['ancho']
    alto = escena['alto']
    obstaculos = escena['obstaculos']
    q0 = escena['q0']
    qf = escena['qf']

    q0_xy = (q0[0], q0[1])
    qf_xy = (qf[0], qf[1])

    path_xy = _rrt(q0_xy, qf_xy, obstaculos, ancho, alto)

    if path_xy is None:
        return None

    path_xy = _suavizar(path_xy, obstaculos, ancho, alto)
    path_xy = _partir_segmentos_largos(path_xy)

    return _camino_a_waypoints(path_xy, q0, qf)
