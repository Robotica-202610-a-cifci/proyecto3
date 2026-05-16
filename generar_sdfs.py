"""
Genera los archivos escena1.sdf ... escena6.sdf para Gazebo
a partir de los archivos Escena-ProblemaX.txt del proyecto.

Uso:
    python3 generar_sdfs.py

Los SDF se guardan en ~/mentorpim1_simulation/
"""
import math
import os

SDF_DIR = os.path.expanduser('~/mentorpim1_simulation')
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')

HEADER = """<?xml version='1.0' encoding='utf-8'?>
<sdf version="1.6">
  <world name="pioneer3_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>"""

FOOTER = """  </world>
</sdf>"""


def parsear_txt(ruta):
    data = {'obstaculos': []}
    with open(ruta, 'r') as f:
        for linea in f:
            partes = [p.strip() for p in linea.strip().split(',')]
            c = partes[0]
            if c == 'Dimensiones':
                data['ancho'] = float(partes[1])
                data['alto'] = float(partes[2])
            elif c == 'q0':
                data['q0'] = (float(partes[1]), float(partes[2]), float(partes[3]))
            elif c == 'qf':
                data['qf'] = (float(partes[1]), float(partes[2]), float(partes[3]))
            elif '_Pto1' in c:
                data['obstaculos'].append({'p1': (float(partes[1]), float(partes[2]))})
            elif '_Pto2' in c:
                data['obstaculos'][-1]['p2'] = (float(partes[1]), float(partes[2]))
    return data


def caja(nombre, cx, cy, cz, sx, sy, sz, color, con_colision=True):
    col = ''
    if con_colision:
        col = f"""
        <collision name="{nombre}_col">
          <pose>{cx} {cy} {cz} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>"""
    return col + f"""
        <visual name="{nombre}_vis">
          <pose>{cx} {cy} {cz} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>"""


def generar_sdf(escena_num):
    ruta_txt = os.path.join(DATA_DIR, f'Escena-Problema{escena_num}.txt')
    if not os.path.exists(ruta_txt):
        print(f'  No encontrado: {ruta_txt}')
        return

    d = parsear_txt(ruta_txt)
    q0 = d['q0']
    qf = d['qf']
    ancho = d['ancho']
    alto = d['alto']
    obs = d['obstaculos']

    # Los últimos 2 obstáculos son los de referencia para relocalización (cyan)
    obs_regular = obs[:-2]
    obs_loc = obs[-2:]

    yaw_q0 = math.radians(q0[2])

    sdf = HEADER

    # Robot en q0
    sdf += f"""
    <include>
      <uri>mentorpi.sdf</uri>
      <pose>{q0[0]} {q0[1]} 0 0 0 {yaw_q0:.6f}</pose>
    </include>"""

    # Indicador q0 (azul)
    sdf += f"""
    <model name="q0_indicator">
      <static>true</static>
      <pose>{q0[0]} {q0[1]} 0.001 0 0 0</pose>
      <link name="q0_indicator_link">
        <visual name="q0_indicator_visual">
          <geometry><plane><size>0.5 0.5</size></plane></geometry>
          <material>
            <ambient>0.5 0.9 0.9 1</ambient>
            <diffuse>0.5 0.9 0.9 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    # Indicador qf (verde)
    sdf += f"""
    <model name="qf_indicator">
      <static>true</static>
      <pose>{qf[0]} {qf[1]} 0.001 0 0 0</pose>
      <link name="qf_indicator_link">
        <visual name="qf_indicator_visual">
          <geometry><plane><size>0.5 0.5</size></plane></geometry>
          <material>
            <ambient>0.5 0.9 0.5 1</ambient>
            <diffuse>0.5 0.9 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    # Obstáculos y paredes
    sdf += """
    <model name="static_environment">
      <static>true</static>
      <link name="walls_link">"""

    # Obstáculos regulares (gris)
    for i, ob in enumerate(obs_regular, 1):
        x1, y1 = ob['p1']
        x2, y2 = ob['p2']
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        sx = abs(x2 - x1)
        sy = abs(y2 - y1)
        sdf += caja(f'obst{i}', cx, cy, 0.25, sx, sy, 0.5, '0.4 0.4 0.4 1')

    # Obstáculos de referencia (cyan)
    for i, ob in enumerate(obs_loc, 1):
        x1, y1 = ob['p1']
        x2, y2 = ob['p2']
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        sx = abs(x2 - x1)
        sy = abs(y2 - y1)
        sdf += caja(f'obstLoc{i}', cx, cy, 0.25, sx, sy, 0.5, '0.1 0.9 0.9 1')

    # Paredes del escenario
    # El obstáculo de referencia derecho cubre x=[4.0,4.2], y=[3.0,5.0]
    # El obstáculo de referencia superior cubre x=[2.0,4.0], y=[5.0,5.2]
    # Las paredes llenan los huecos restantes

    loc_r = obs_loc[0]  # obstáculo referencia derecho
    loc_u = obs_loc[1]  # obstáculo referencia superior

    # Coordenadas de los obstáculos de referencia
    loc_r_ymin = min(loc_r['p1'][1], loc_r['p2'][1])
    loc_u_xmin = min(loc_u['p1'][0], loc_u['p2'][0])

    # Pared inferior (full ancho)
    sdf += caja('box_down', ancho/2, -0.05, 0.25, ancho, 0.1, 0.5, '0.55 0.4 0.4 1')

    # Pared izquierda (full alto)
    sdf += caja('box_left', -0.05, alto/2, 0.25, 0.1, alto, 0.5, '0.55 0.4 0.4 1')

    # Pared derecha (solo hasta donde empieza obstLoc1)
    if loc_r_ymin > 0:
        sdf += caja('box_right', ancho + 0.05, loc_r_ymin/2, 0.25,
                    0.1, loc_r_ymin, 0.5, '0.55 0.4 0.4 1')

    # Pared superior (solo hasta donde empieza obstLoc2)
    if loc_u_xmin > 0:
        sdf += caja('box_up', loc_u_xmin/2, alto + 0.05, 0.25,
                    loc_u_xmin, 0.1, 0.5, '0.55 0.4 0.4 1')

    sdf += """
      </link>
    </model>"""

    sdf += '\n' + FOOTER

    # Guardar
    ruta_sdf = os.path.join(SDF_DIR, f'escena{escena_num}.sdf')
    with open(ruta_sdf, 'w') as f:
        f.write(sdf)
    print(f'  Generado: {ruta_sdf}')


if __name__ == '__main__':
    print(f'Guardando SDFs en: {SDF_DIR}')
    for n in range(1, 7):
        generar_sdf(n)
    print('Listo.')
