import math

def obtener_distancia_angulo(msg_scan, angulo_objetivo_rad):
    """
    Calcula la distancia a un obstáculo en un ángulo específico.
    """
    if msg_scan is None: return float('inf')
    
    # Encontrar el índice correspondiente al ángulo objetivo
    # Normalizamos el ángulo basado en las propiedades del mensaje Lidar
    indice = int((angulo_objetivo_rad - msg_scan.angle_min) / msg_scan.angle_increment)
    
    # Validamos que el índice no se salga de los límites del arreglo
    if 0 <= indice < len(msg_scan.ranges):
        distancia = msg_scan.ranges[indice]
        # Verificamos que la lectura sea confiable (ni muy cerca ni "infinita")
        if msg_scan.range_min <= distancia <= msg_scan.range_max:
            return distancia
            
    return float('inf')

def obtener_distancias_rango(msg_scan, angulo_min_deg, angulo_max_deg):
    """
    Retorna una lista con las distancias válidas capturadas en un rango de ángulos (en grados).
    """
    if msg_scan is None: return []

    rad_min = math.radians(angulo_min_deg)
    rad_max = math.radians(angulo_max_deg)
    distancias = []

    for i, dist in enumerate(msg_scan.ranges):
        angulo = msg_scan.angle_min + (i * msg_scan.angle_increment)
        
        # Filtramos por el rango de visión deseado (ej. -50 a 50 grados)
        if rad_min <= angulo <= rad_max:
            if msg_scan.range_min <= dist <= msg_scan.range_max:
                distancias.append(dist)
                
    return distancias