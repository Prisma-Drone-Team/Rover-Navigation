#!/bin/bash

if [ -z "$1" ]; then
    echo "Errore: Inserisci il nome del robot"
    exit 1
fi

echo "Attendere almeno 30 secondi dopo l'avvio della simulazione prima di eseguire lo script."
echo ""

echo "[1/10] Misurazione Real-Time Factor (RTF Vero - Media 5s)..."
cat << 'EOF' > /tmp/rtf_calc.py
import rclpy, time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class RTFCalculator(Node):
    def __init__(self):
        super().__init__('rtf_calc')
        self.sub = self.create_subscription(Clock, '/clock', self.cb, 10)
        self.start_sim = None
        
    def cb(self, msg):
        sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.start_sim is None:
            self.start_sim = sim_t
            self.start_real = time.time()
            return
            
        real_elapsed = time.time() - self.start_real
        if real_elapsed >= 5.0:
            sim_elapsed = sim_t - self.start_sim
            print(f"RTF Medio: {sim_elapsed/real_elapsed:.3f}x")
            raise SystemExit

rclpy.init()
try:
    rclpy.spin(RTFCalculator())
except SystemExit:
    pass
except KeyboardInterrupt:
    pass
finally:
    rclpy.shutdown()
EOF
timeout 7 python3 /tmp/rtf_calc.py 2>/dev/null || echo "Nessun dato su /clock ricevuto."
echo ""

echo "[2/10] Misurazione Frequenza /clock..."
timeout 5 ros2 topic hz /clock > /tmp/clock_test.txt 2>/dev/null
grep "average rate" /tmp/clock_test.txt | tail -n 1
echo ""

echo "[3/10] Misurazione Frequenza Odometria (Ruote)..."
timeout 5 ros2 topic hz /$1/odom > /tmp/odom_test.txt 2>/dev/null
grep "average rate" /tmp/odom_test.txt | tail -n 1
echo ""

echo "[4/10] Misurazione Frequenza Depth Camera..."
timeout 5 ros2 topic hz /$1/depth/image_raw/points > /tmp/depth_hz_test.txt 2>/dev/null
grep "average rate" /tmp/depth_hz_test.txt | tail -n 1
echo ""

echo "[5/10] Misurazione Frequenza RGB Camera..."
timeout 5 ros2 topic hz /$1/color/image_raw > /tmp/rgb_hz_test.txt 2>/dev/null
grep "average rate" /tmp/rgb_hz_test.txt | tail -n 1
echo ""

echo "[6/10] Misurazione Larghezza di Banda PointCloud..."
timeout 5 ros2 topic bw /$1/depth/image_raw/points > /tmp/depth_bw_test.txt 2>/dev/null
tail -n 2 /tmp/depth_bw_test.txt
echo ""

echo "[7/10] Misurazione Frequenza RTAB-Map (Mappa 3D)..."
timeout 15 ros2 topic hz /$1/cloud_map > /tmp/cloudmap_test.txt 2>/dev/null
grep "average rate" /tmp/cloudmap_test.txt | tail -n 1
echo ""

echo "[8/10] Misurazione Frequenza Nav2 (Local Costmap)..."
timeout 15 ros2 topic hz /$1/local_costmap/costmap > /tmp/costmap_test.txt 2>/dev/null
grep "average rate" /tmp/costmap_test.txt | tail -n 1
echo ""

echo "[9/10] Misurazione Ritardo TF Dinamici..."
timeout 12 ros2 run tf2_ros tf2_monitor --ros-args -p use_sim_time:=true > /tmp/tf_test.txt 2>/dev/null
grep -E "base_footprint|odom|rocker|wheel" /tmp/tf_test.txt | head -n 5
echo "..."
echo ""

echo "[10/10] Consumo Risorse Sistema (CPU e RAM)..."
CPU_USE=$(top -b -n 2 -d 0.5 | grep "%Cpu" | tail -n 1 | awk '{print $2 + $4}')
echo "CPU TOTALE del computer: ${CPU_USE}%"
RAM_USE=$(free -h | awk '/Mem:/ {print $3 " usati su " $2}')
echo "Memoria RAM TOTALE: ${RAM_USE}"
echo ""

echo "--- Dettaglio Consumi per Singolo Processo ---"
print_process_stats() {
    local proc_name=$1
    local display_name=$2
    local pids=$(pgrep -d ',' -f "$proc_name")
    
    if [ -n "$pids" ]; then
        local stats=$(ps -p $pids -o %cpu=,%mem= | awk '{cpu+=$1; mem+=$2} END {print cpu, mem}')
        local cpu=$(echo "$stats" | awk '{print $1}')
        local mem=$(echo "$stats" | awk '{print $2}')
        printf "%-20s: CPU %5.1f%% | RAM %5.1f%%\n" "$display_name" "$cpu" "$mem"
    else
        printf "%-20s: Non in esecuzione\n" "$display_name"
    fi
}

print_process_stats "webots-bin" "Simulatore Webots"
print_process_stats "ign gazebo" "Simulatore Gazebo"
print_process_stats "rtabmap" "RTAB-Map (SLAM)"
print_process_stats "planner_server" "Nav2 (Global Plan)"
print_process_stats "controller_server" "Nav2 (Local Plan)"
print_process_stats "rover_manager" "Rover Manager"

echo ""