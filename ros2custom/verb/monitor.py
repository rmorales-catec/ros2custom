import argparse
import subprocess
import threading
import re
import time
import os
from ros2cli.verb import VerbExtension
from tabulate import tabulate

class MonitorVerb(VerbExtension):
    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        parser.add_argument('topic', help='Nombre del tópico a monitorear')
        parser.add_argument('-qos', '--QualityOfService', action='store_true', help='Muestra información detallada del QoS')
        parser.add_argument('-net', '--NetworkInfo', action='store_true', help='Muestra estadísticas de red incluyendo loopback')
        parser.add_argument('-w', '--window', type=int, default=1000, help='Tamaño de la ventana para los cálculos de hz, delay y bw')


    def main(self, *, args):
        topic = args.topic
        qos = args.QualityOfService
        net = args.NetworkInfo

        data = {
            'hz': '-', 'hz_win': '-',
            'delay': '-', 'delay_win': '-',
            'bw': '-', 'msg_size': '-'
        }


        qos_summary = ""
        def parse_hz(line):
            if "average rate" in line:
                m = re.search(r'average rate:\s+([\d.]+)', line)
                if m:
                    data['hz'] = m.group(1)
            elif "window:" in line:
                m = re.search(r'window:\s+(\d+)', line)
                if m:
                    data['hz_win'] = m.group(1)

        def parse_delay(line):
            if "average delay" in line:
                m = re.search(r'average delay:\s+([\d.]+)', line)
                if m:
                    data['delay'] = m.group(1)
            elif "window:" in line:
                m = re.search(r'window:\s+(\d+)', line)
                if m:
                    data['delay_win'] = m.group(1)

        def parse_bw(line):
            m = re.search(r'([0-9.]+)\s*(B|KB|MB)/s\s+from', line)
            if m:
                bw = float(m.group(1))
                unit = m.group(2).upper()
                if unit == 'B':
                    bw /= 1024.0 * 1024.0
                elif unit == 'KB':
                    bw /= 1024.0
                data['bw'] = round(bw, 2)
            m = re.search(r'Message size mean:\s+([0-9.]+)\s*(B|KB|MB)', line)
            if m:
                msg_size = float(m.group(1))
                unit = m.group(2).upper()
                if unit == 'B':
                    msg_size /= 1024.0 * 1024.0
                elif unit == 'KB':
                    msg_size /= 1024.0
                data['msg_size'] = round(msg_size, 2)

        def launch_monitor(command, parser_fn):
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            for line in proc.stdout:
                parser_fn(line)


        interface_data = {}
        net_lock = threading.Lock()
        if net:
            def format_bits(value_bits):
                kb = 1024
                mb = kb * 1024
                gb = mb * 1024
                if value_bits < kb:
                    return f"{value_bits:.2f} b/s"
                elif value_bits < mb:
                    return f"{value_bits / kb:.2f} Kb/s"
                elif value_bits < gb:
                    return f"{value_bits / mb:.2f} Mb/s"
                else:
                    return f"{value_bits / gb:.2f} Gb/s"

            def monitor_network():
                old_data = {}
                while True:
                    time.sleep(1)
                    with open("/proc/net/dev", "r") as f:
                        lines = f.readlines()[2:]

                    with net_lock:
                        for line in lines:
                            parts = line.strip().split()
                            iface = parts[0].strip(":")
                            rx_bytes = int(parts[1])
                            tx_bytes = int(parts[9])

                            rx_bits = rx_bytes * 8
                            tx_bits = tx_bytes * 8

                            if iface in old_data:
                                delta_rx = rx_bits - old_data[iface]['rx']
                                delta_tx = tx_bits - old_data[iface]['tx']
                                interface_data[iface] = {
                                    'rx': format_bits(delta_rx),
                                    'tx': format_bits(delta_tx)
                                }
                            old_data[iface] = {'rx': rx_bits, 'tx': tx_bits}
            threading.Thread(target=monitor_network, daemon=True).start()


        threading.Thread(target=launch_monitor, args=(['ros2', 'topic', 'hz', topic, '-w', str(args.window)], parse_hz), daemon=True).start()
        threading.Thread(target=launch_monitor, args=(['ros2', 'topic', 'delay', topic, '-w', str(args.window)], parse_delay), daemon=True).start()
        threading.Thread(target=launch_monitor, args=(['ros2', 'topic', 'bw', topic, '-w', str(args.window)], parse_bw), daemon=True).start()


        if qos:
            qos_summary_lock = threading.Lock()
            qos_summary_lock = threading.Lock()

            def update_qos():
                nonlocal qos_summary
                while True:
                    try:
                        raw_output = subprocess.check_output(['ros2', 'topic', 'info', '-v', topic], text=True)
                        with qos_summary_lock:
                            qos_summary = raw_output
                    except subprocess.CalledProcessError:
                        with qos_summary_lock:
                            qos_summary = "[ERROR] No se pudo obtener la información QoS."

                    time.sleep(2)

            threading.Thread(target=update_qos, daemon=True).start()

        try:
            while True:
                table = [
                    ["Valor", f"{data['hz']} hz", f"{data['delay']} s", f"{data['bw']} MB/s"],
                    ["Info", f"win: {data['hz_win']}", f"win: {data['delay_win']}", f"msg_size: {data['msg_size']} MB"],
                ]
                headers = [' ', 'HZ', 'Delay', 'BW']
                print(tabulate(table, headers=headers, tablefmt='fancy_grid'))

                if net:
                    print("\n[Network Info]")
                    with net_lock:
                        net_table = [["Interfaz", "RX", "TX"]]
                        for iface, stats in sorted(interface_data.items()):
                            net_table.append([iface, stats['rx'], stats['tx']])
                    print(tabulate(net_table, headers="firstrow", tablefmt='fancy_grid'))

                if qos:
                    print()
                    with qos_summary_lock:
                        print(qos_summary)
                time.sleep(1)

                print("\n" + "="*80 + "\n")

        except KeyboardInterrupt:
            print("\nMonitor detenido.")
