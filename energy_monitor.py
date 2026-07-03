#!/usr/bin/env python3
"""
energy_monitor.py -  V2 energy-aware autonomy

Monitora il percorso del rover via TF (map -> frame base) e integra una stima
di energia consumata:

    energia = c_roll * distanza_orizzontale  +  c_climb * salita_positiva

L'energia viene calcolata tenendo conto della distanza percorsa a cui si aggiunge l'effetto pendenza: salire costa di piu', quindi il
terreno ripido scarica il budget piu' in fretta. 
E' il legame V2<->V1: la stessa inclinazione del terreno che V1 penalizza nella SlopeCostMap qui penalizza il budget. 
Su terreno piatto (dz~0) degrada a sola-distanza.
Alza c_climb per dare piu' peso alla pendenza, oppure mettilo a 0 per un modello a sola distanza.

Quando l'energia cumulata raggiunge il budget, pubblica il fatto simbolico
'battery_low' sul topic di stato SEED del rover, in questo modo il livello cognitivo fa scattare il rientro anticipato alla base.

Nel rover_container:
  python3 energy_monitor.py --ros-args \
      -p budget:=40.0 -p c_roll:=1.0 -p c_climb:=8.0 \
      -p map_frame:=map -p base_frame:=robot1/base_footprint \
      -p state_topic:=/seed_pdt_rover1/state
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ConnectivityException, ExtrapolationException)


class EnergyMonitor(Node):
    def __init__(self):
        super().__init__('energy_monitor')

        #parametri 
        self.declare_parameter('budget', 40.0)       # budget energetico 
        self.declare_parameter('c_roll', 1.0)        # energia per metro percorso
        self.declare_parameter('c_climb', 8.0)       # energia per metro di salita positiva
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'robot1/base_footprint')
        self.declare_parameter('state_topic', '/seed_pdt_rover1/state')
        self.declare_parameter('fact', 'battery_low')
        self.declare_parameter('sample_rate', 5.0)   # Hz
        self.declare_parameter('min_step', 0.02)     # m, ignora il jitter del TF sotto questa soglia

        self.budget = self.get_parameter('budget').value
        self.c_roll = self.get_parameter('c_roll').value
        self.c_climb = self.get_parameter('c_climb').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.state_topic = self.get_parameter('state_topic').value
        self.fact = self.get_parameter('fact').value
        self.min_step = self.get_parameter('min_step').value
        rate = self.get_parameter('sample_rate').value

        # stato 
        self.energy = 0.0
        self.distance = 0.0
        self.climb = 0.0
        self.prev = None          # (x, y, z) campione precedente
        self.fired = False

        # interfacce ROS 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(String, self.state_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"energy_monitor active | budget={self.budget} c_roll={self.c_roll} "
            f"c_climb={self.c_climb} | {self.map_frame}->{self.base_frame} "
            f"| fatto '{self.fact}' -> {self.state_topic}")

    def tick(self):
        if self.fired:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return  # TF not ready

        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z

        if self.prev is None:
            self.prev = (x, y, z)
            return

        dx = x - self.prev[0]
        dy = y - self.prev[1]
        dz = z - self.prev[2]
        dhoriz = math.hypot(dx, dy)

        if dhoriz >= self.min_step:
            climb = max(0.0, dz)               # solo la salita costa di piu'
            self.distance += dhoriz
            self.climb += climb
            self.energy += self.c_roll * dhoriz + self.c_climb * climb
            self.prev = (x, y, z)
            self.get_logger().info(
                f"E={self.energy:6.2f}/{self.budget:.0f}  "
                f"dist={self.distance:6.2f}m  salita={self.climb:5.2f}m")

        if self.energy >= self.budget and not self.fired:
            self.fire()

    def fire(self):
        self.fired = True
        msg = String()
        msg.data = self.fact
        for _ in range(3):                     # invio ridondante, come in A3
            self.pub.publish(msg)
        self.get_logger().warn(
            f"BUDGET RAGGIUNTO  E={self.energy:.2f} "
            f"(dist={self.distance:.2f}m salita={self.climb:.2f}m) "
            f"-> pubblicato '{self.fact}' su {self.state_topic}")


def main():
    rclpy.init()
    node = EnergyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
