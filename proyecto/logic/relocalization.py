"""
Relocalización del robot al llegar a qf.

El robot mide con LiDAR las distancias reales al obstáculo de referencia
al frente y a la derecha. Usando la posición teórica de esos obstáculos
(calculada a partir de qf y las distancias teóricas del enunciado) se
recalcula la posición real del robot: qact.
"""
import math


def calcular_qact(qf, d_frente_teorico, d_derecha_teorico,
                  d_frente_medido, d_derecha_medido, theta_est_rad):
    """
    Calcula la configuración real qact.

    Parámetros
    ----------
    qf                : (x, y, theta_grados)  – configuración final teórica
    d_frente_teorico  : float  – distancia teórica al obstáculo frontal (m)
    d_derecha_teorico : float  – distancia teórica al obstáculo derecho (m)
    d_frente_medido   : float  – distancia LiDAR medida al frente (m)
    d_derecha_medido  : float  – distancia LiDAR medida a la derecha (m)
    theta_est_rad     : float  – orientación estimada por odometría (rad)

    Retorna
    -------
    (x_act, y_act, theta_grados_act)
    """
    qf_x, qf_y, qf_theta_deg = qf
    qf_theta_rad = math.radians(qf_theta_deg)

    # --- Vectores unitarios según orientación teórica de qf ---
    # frente: dirección del heading del robot
    frente_dx = math.cos(qf_theta_rad)
    frente_dy = math.sin(qf_theta_rad)
    # derecha: 90° en sentido horario respecto al frente
    derecha_dx = math.sin(qf_theta_rad)
    derecha_dy = -math.cos(qf_theta_rad)

    # --- Posición de los obstáculos de referencia (desde qf teórico) ---
    obs_frente_x = qf_x + d_frente_teorico * frente_dx
    obs_frente_y = qf_y + d_frente_teorico * frente_dy
    obs_derecha_x = qf_x + d_derecha_teorico * derecha_dx
    obs_derecha_y = qf_y + d_derecha_teorico * derecha_dy

    # --- Vectores según orientación estimada (odometría) ---
    frente_act_dx = math.cos(theta_est_rad)
    frente_act_dy = math.sin(theta_est_rad)
    derecha_act_dx = math.sin(theta_est_rad)
    derecha_act_dy = -math.cos(theta_est_rad)

    # --- qact desde obstáculo frontal ---
    qact_x_f = obs_frente_x - d_frente_medido * frente_act_dx
    qact_y_f = obs_frente_y - d_frente_medido * frente_act_dy

    # --- qact desde obstáculo derecho ---
    qact_x_d = obs_derecha_x - d_derecha_medido * derecha_act_dx
    qact_y_d = obs_derecha_y - d_derecha_medido * derecha_act_dy

    # Promedio de las dos estimaciones
    qact_x = (qact_x_f + qact_x_d) / 2.0
    qact_y = (qact_y_f + qact_y_d) / 2.0
    qact_theta_deg = math.degrees(theta_est_rad)

    return (qact_x, qact_y, qact_theta_deg)


def distancia_posicion(q1_xy, q2_xy):
    """Distancia euclidea entre dos posiciones (x, y)."""
    return math.sqrt((q1_xy[0] - q2_xy[0]) ** 2 + (q1_xy[1] - q2_xy[1]) ** 2)


def diferencia_angular_deg(theta1_deg, theta2_deg):
    """Diferencia angular mínima en grados (−180 a 180)."""
    diff = theta1_deg - theta2_deg
    diff = math.degrees(math.atan2(math.sin(math.radians(diff)),
                                   math.cos(math.radians(diff))))
    return diff
