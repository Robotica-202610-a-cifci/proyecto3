import math
from geometry_msgs.msg import Twist

VEL_LINEAL = 0.20
VEL_ANGULAR = 0.5
VEL_ANGULAR_MIN = 0.18   # minimo para vencer friccion estatica del mecanum
TOL_DIST = 0.20
TOL_ANGLE = 0.10         # ~6 grados, suficiente para navegacion real


class PathExecutor:
    _ROTATE = 'ROTATE'
    _TRANSLATE = 'TRANSLATE'

    def __init__(self, waypoints):
        self.waypoints = waypoints
        # Empezamos en idx=1: el robot ya esta fisicamente en q0 (waypoints[0]),
        # asi que saltamos ese waypoint redundante y vamos directo a la primera
        # rotacion hacia el camino planificado.
        self.idx = 1 if len(waypoints) > 1 else 0
        self.phase = self._ROTATE

    def tick(self, current_x, current_y, current_theta_rad, node):
        if self.idx >= len(self.waypoints):
            self._detener(node)
            return 'COMPLETADO'

        tx, ty, ttheta_deg = self.waypoints[self.idx]
        ttheta_rad = math.radians(ttheta_deg)

        # ---- ROTACION ----
        if self.phase == self._ROTATE:
            error_ang = math.atan2(math.sin(ttheta_rad - current_theta_rad),
                                   math.cos(ttheta_rad - current_theta_rad))
            if abs(error_ang) < TOL_ANGLE:
                self._detener(node)
                self.phase = self._TRANSLATE
                return 'EN_RUTA'
            # Velocidad proporcional con minimo para vencer friccion estatica
            w = 2.0 * error_ang
            if abs(w) < VEL_ANGULAR_MIN:
                w = math.copysign(VEL_ANGULAR_MIN, error_ang)
            cmd = Twist()
            cmd.angular.z = max(-VEL_ANGULAR, min(VEL_ANGULAR, w))
            node.cmd_pub.publish(cmd)
            return 'EN_RUTA'

        # ---- TRASLACION con correccion suave ----
        if self.phase == self._TRANSLATE:
            dx = tx - current_x
            dy = ty - current_y
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist < TOL_DIST:
                self._detener(node)
                self._advance()
                return 'EN_RUTA'

            angulo_objetivo = math.atan2(dy, dx)
            error_ang = math.atan2(math.sin(angulo_objetivo - current_theta_rad),
                                   math.cos(angulo_objetivo - current_theta_rad))
            vel = max(0.06, min(VEL_LINEAL, 0.4 * dist))
            cmd = Twist()
            cmd.linear.x = vel
            # Solo corrige si el error supera 8 grados para evitar oscilacion
            cmd.angular.z = max(-0.2, min(0.2, 0.5 * error_ang)) if abs(error_ang) > 0.14 else 0.0
            node.cmd_pub.publish(cmd)
            return 'EN_RUTA'

        return 'COMPLETADO'

    def _detener(self, node):
        node.cmd_pub.publish(Twist())

    def _advance(self):
        self.idx += 1
        self.phase = self._ROTATE

    @property
    def terminado(self):
        return self.idx >= len(self.waypoints)

    @property
    def progreso(self):
        return f'{self.idx}/{len(self.waypoints)}'
