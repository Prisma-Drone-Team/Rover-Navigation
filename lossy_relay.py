#!/usr/bin/env python3
"""
Nodo di iniezione perdita per il test A3 (tolleranza alla perdita di pacchetti).

Ascolta il canale di notifica "lossy" di leo1 e lo reinoltra al topic di stato
reale di leo2, scartando ogni messaggio con probabilita' = drop_rate.
Conta inoltrati/droppati per la misura del tasso di consegna.

Uso tipico (drop al 30%):
    python3 lossy_relay.py --ros-args -p drop_rate:=0.3

Routing del test: nei tre schemi send_rockg_K di leo1, manda la notifica a
'seed_pdt_rover2/state_lossy' (input di questo nodo). leo2 resta invariato e
continua ad ascoltare 'seed_pdt_rover2/state' (output di questo nodo).
"""

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as MSG_TYPE

class LossyRelay(Node):
    def __init__(self):
        super().__init__("lossy_relay")

        self.declare_parameter("drop_rate", 0.0)
        self.declare_parameter("input_topic", "seed_pdt_rover2/state_lossy")
        self.declare_parameter("output_topic", "seed_pdt_rover2/state")

        self.drop_rate = float(self.get_parameter("drop_rate").value)
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(MSG_TYPE, out_topic, 10)
        self.sub = self.create_subscription(MSG_TYPE, in_topic, self.cb, 10)

        self.n_in = 0 #number of input messages
        self.n_fwd = 0 #number of output messages 

        self.get_logger().info(
            f"LossyRelay active: {in_topic} -> {out_topic} | drop_rate={self.drop_rate:.2f}"
        )

    def cb(self, msg):
        self.n_in += 1
        if random.random() >= self.drop_rate:
            self.pub.publish(msg)
            self.n_fwd += 1
            self.get_logger().info(
                f"SUBMITTED  (SUBMITTED {self.n_fwd}/{self.n_in}, "
                f"DELIVERY {100.0 * self.n_fwd / self.n_in:.0f}%)"
            )
        else:
            self.get_logger().info(
                f"DROPPED   (SUBMETTED {self.n_fwd}/{self.n_in}, "
                f"DELIVERY {100.0 * self.n_fwd / self.n_in:.0f}%)"
            )


def main():
    rclpy.init()
    node = LossyRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
