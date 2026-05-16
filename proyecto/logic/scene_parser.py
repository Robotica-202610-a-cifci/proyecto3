import math


def parsear_escena(texto_escena):
    """Parsea el texto de una escena y retorna un dict con toda la info."""
    data = {'obstaculos': []}
    lineas = texto_escena.strip().split('\n')

    for linea in lineas:
        linea = linea.strip()
        if not linea:
            continue
        partes = [p.strip() for p in linea.split(',')]
        clave = partes[0]

        if clave == 'Dimensiones':
            data['ancho'] = float(partes[1])
            data['alto'] = float(partes[2])
        elif clave == 'q0':
            data['q0'] = (float(partes[1]), float(partes[2]), float(partes[3]))
        elif clave == 'qf':
            data['qf'] = (float(partes[1]), float(partes[2]), float(partes[3]))
        elif clave == 'dFrente':
            data['dFrente'] = float(partes[1])
        elif clave == 'dDerecha':
            data['dDerecha'] = float(partes[1])
        elif clave == 'Obstaculos':
            data['num_obstaculos'] = int(partes[1])
        elif '_Pto1' in clave:
            data['obstaculos'].append({
                'pto1': (float(partes[1]), float(partes[2]))
            })
        elif '_Pto2' in clave:
            data['obstaculos'][-1]['pto2'] = (float(partes[1]), float(partes[2]))

    return data


def leer_camino(ruta_archivo):
    """Lee un archivo de camino .txt y retorna lista de (x, y, theta_grados)."""
    waypoints = []
    with open(ruta_archivo, 'r', encoding='utf-8') as f:
        for linea in f:
            linea = linea.strip()
            if not linea or linea.startswith('#'):
                continue
            partes = linea.split(',')
            if len(partes) >= 3:
                try:
                    x = float(partes[0])
                    y = float(partes[1])
                    theta = float(partes[2])
                    waypoints.append((x, y, theta))
                except ValueError:
                    pass
    return waypoints


def guardar_camino(ruta_archivo, waypoints, qf_est=None, qact=None):
    """Guarda el camino en .txt. Opcionalmente añade qf-est y qact al final."""
    with open(ruta_archivo, 'w', encoding='utf-8') as f:
        for (x, y, theta) in waypoints:
            f.write(f'{x:.4f},{y:.4f},{theta:.4f}\n')
        if qf_est is not None:
            f.write(f'# qf-est: {qf_est[0]:.4f},{qf_est[1]:.4f},{math.degrees(qf_est[2]):.4f}\n')
        if qact is not None:
            f.write(f'# qact: {qact[0]:.4f},{qact[1]:.4f},{qact[2]:.4f}\n')
