#!/usr/bin/env python3
"""
energy_logger.py — registra su CSV l'energia cumulata e la distanza percorsa,
usando lo STESSO modello di energy_monitor.py:
    E = c_roll * distanza_orizzontale + c_climb * dislivello_positivo

Va lanciato ACCANTO allo scenario S2 (dentro rover_container): registra e basta,
NON pubblica battery_low (quello resta compito di energy_monitor.py).

Esempio (run interrupt, budget 40 sul monitor):
  python3 energy_logger.py --ros-args \
    -p c_roll:=1.0 -p c_climb:=8.0 \
    -p map_frame:=robot1/map -p base_frame:=robot1/base_footprint \
    -p csv_path:=energy_interrupt.csv

Per la run di controllo (budget alto sul monitor) cambia solo:
    -p csv_path:=energy_control.csv
"""
import csv, math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class EnergyLogger(Node):
    def __init__(self):
        super().__init__('energy_logger')
        self.declare_parameter('c_roll', 1.0)
        self.declare_parameter('c_climb', 8.0)
        self.declare_parameter('map_frame', 'robot1/map')
        self.declare_parameter('base_frame', 'robot1/base_footprint')
        self.declare_parameter('csv_path', 'energy_run.csv')
        self.declare_parameter('rate', 10.0)

        self.c_roll = self.get_parameter('c_roll').value
        self.c_climb = self.get_parameter('c_climb').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.csv_path = self.get_parameter('csv_path').value
        rate = self.get_parameter('rate').value

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.prev = None
        self.dist = 0.0
        self.climb = 0.0
        self.t0 = None

        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'z', 'distance', 'climb', 'energy'])
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'energy_logger -> {self.csv_path} '
                               f'(c_roll={self.c_roll}, c_climb={self.c_climb})')

    def tick(self):
        try:
            tr = self.buf.lookup_transform(self.map_frame, self.base_frame,
                                           rclpy.time.Time())
        except Exception:
            return  # TF non ancora disponibile: salta il campione
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        z = tr.transform.translation.z
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        if self.prev is not None:
            px, py, pz = self.prev
            self.dist += math.hypot(x - px, y - py)
            dz = z - pz
            if dz > 0.0:
                self.climb += dz
        self.prev = (x, y, z)
        energy = self.c_roll * self.dist + self.c_climb * self.climb
        self.w.writerow([f'{now - self.t0:.3f}', f'{x:.4f}', f'{y:.4f}',
                         f'{z:.4f}', f'{self.dist:.4f}', f'{self.climb:.4f}',
                         f'{energy:.4f}'])
        self.f.flush()

    def destroy_node(self):
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = EnergyLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
