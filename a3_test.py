#!/usr/bin/env python3
"""
Harness di test per A3 (tolleranza alla perdita di pacchetti).

Isola la sola fase di notifica: pubblica N copie ridondanti di un messaggio
attraverso il canale con perdita (il relay) e conta in quante prove almeno
una copia arriva a destinazione. Restituisce il tasso di consegna.

Replica esattamente cio' che fa il meccanismo A3 nell'LTM (leo1 invia
rockg_found N volte spaziate), ma in modo ripetibile e veloce.

Va eseguito con il relay attivo e SENZA la missione completa in corso
(servono solo relay + harness, per misurare il canale in isolamento).

Esempio (triplo invio, 100 prove):
    python3 a3_test_harness.py --ros-args -p n_copies:=3 -p n_trials:=100
Baseline (invio singolo):
    python3 a3_test_harness.py --ros-args -p n_copies:=1 -p n_trials:=100
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class A3TestHarness(Node):
    def __init__(self):
        super().__init__("a3_test_harness")

        self.declare_parameter("n_copies", 3)            # copie per delega (1 = baseline)
        self.declare_parameter("n_trials", 100)          # numero di prove
        self.declare_parameter("copy_spacing", 0.1)      # s tra copie ridondanti
        self.declare_parameter("trial_timeout", 0.5)     # s di attesa consegna per prova
        self.declare_parameter("input_topic", "seed_pdt_rover2/state_lossy")
        self.declare_parameter("output_topic", "seed_pdt_rover2/state")

        self.n_copies = int(self.get_parameter("n_copies").value)
        self.n_trials = int(self.get_parameter("n_trials").value)
        self.copy_spacing = float(self.get_parameter("copy_spacing").value)
        self.trial_timeout = float(self.get_parameter("trial_timeout").value)
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(String, in_topic, 10)
        self.sub = self.create_subscription(String, out_topic, self.cb, 10)
        self.last_received = None

    def cb(self, msg):
        self.last_received = msg.data

    def _spin(self, dur):
        end = time.time() + dur
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def run(self):
        # pausa per far stabilire le connessioni
        self._spin(1.0)
        successes = 0
        for i in range(self.n_trials):
            tag = f"trial_{i}"
            self.last_received = None
            # invia N copie spaziate (la delega ridondante)
            for _ in range(self.n_copies):
                m = String()
                m.data = tag
                self.pub.publish(m)
                self._spin(self.copy_spacing)
            # finestra di attesa per la consegna
            self._spin(self.trial_timeout)
            if self.last_received == tag:
                successes += 1
        rate = 100.0 * successes / self.n_trials
        self.get_logger().info(
            f"RESULT: n_copies={self.n_copies}, prove={self.n_trials}, "
            f"successes={successes} -> DELIVERY {rate:.1f}%"
        )


def main():
    rclpy.init()
    node = A3TestHarness()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
