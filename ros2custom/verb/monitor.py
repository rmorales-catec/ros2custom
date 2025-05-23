import argparse
import subprocess
import threading
import re
import time
import os
from ros2cli.verb import VerbExtension
from tabulate import tabulate

class MonitorVerb(VerbExtension):
    """Monitorea un topic en tiempo real con HZ, Delay y BW."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        parser.add_argument('topic', help='Nombre del tópico a monitorear')
        parser.add_argument('-qos', '--QualityOfService', action='store_true', help='Muestra información detallada del QoS de cada publisher/subscriber del topic')
        # parser.add_argument('-v', '--verbose', action='store_true', help='Muestra información detallada del QoS del tópico')

    def main(self, *, args):
        topic = args.topic
        qos=args.QualityOfService
        # verbose = args.verbose

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
                    bw = bw / (1024.0 * 1024.0)
                elif unit == 'KB':
                    bw = bw / 1024.0
                data['bw'] = round(bw, 2)
            m = re.search(r'Message size mean:\s+([0-9.]+)\s*(B|KB|MB)', line)
            if m:
                msg_size = float(m.group(1))
                unit = m.group(2).upper()
                if unit == 'B':
                    msg_size = msg_size / (1024.0 * 1024.0)
                elif unit == 'KB':
                    msg_size = msg_size / 1024.0
                data['msg_size'] = round(msg_size, 2)

        def launch_monitor(command, parser_fn):
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            for line in proc.stdout:
                parser_fn(line)

        # Iniciar hilos
        threading.Thread(target=launch_monitor, args=(['ros2', 'topic', 'hz', topic], parse_hz), daemon=True).start()
        threading.Thread(target=launch_monitor, args=(['ros2', 'topic', 'delay', topic], parse_delay), daemon=True).start()
        threading.Thread(target=launch_monitor, args=(['ros2', 'topic', 'bw', topic], parse_bw), daemon=True).start()

        if qos:
            try:
                raw_output = subprocess.check_output(['ros2', 'topic', 'info', '-v', topic], text=True)

                # Extraer tipo de mensaje
                msg_type_match = re.search(r'^Type:\s+(.*)', raw_output, re.MULTILINE)
                msg_type = msg_type_match.group(1) if msg_type_match else "Desconocido"

                blocks = raw_output.split("Node name:")
                parsed = []
                for block in blocks[1:]:
                    lines = block.strip().splitlines()
                    node = endpoint = ""
                    qos = {}
                    for line in lines:
                        if line.startswith("Node name:"):
                            node = line.split(":", 1)[1].strip()
                        elif "Endpoint type:" in line:
                            endpoint = line.split(":", 1)[1].strip()
                        elif ":" in line and "QoS profile" not in line:
                            key, val = line.strip().split(":", 1)
                            qos[key.strip()] = val.strip()
                    parsed.append((node, endpoint, qos))

                # Formatear resumen
                qos_lines = [f"[QoS Info] \n Topic Type: {msg_type}"]
                for node, ep_type, qos in parsed:
                    qos_lines.append(f"  Node: {node} ({ep_type})")
                    for k, v in qos.items():
                        qos_lines.append(f"    {k}: {v}")
                    qos_lines.append("")
                qos_summary = "\n".join(qos_lines)

            except subprocess.CalledProcessError:
                qos_summary = "[ERROR] No se pudo obtener la información QoS."

        try:
            while True:
                os.system('clear')  # en Windows usar 'cls'
                table = [
                    ["Valor", f"{data['hz']} hz", f"{data['delay']} s", f"{data['bw']} MB/s"],
                    ["Info", f"win: {data['hz_win']}", f"win: {data['delay_win']}", f"msg_size: {data['msg_size']} MB"],
                ]
                headers = [' ', 'HZ', 'Delay', 'BW']
                print(tabulate(table, headers=headers, tablefmt='fancy_grid'))

                if qos:
                    print()
                    print(qos_summary)

                time.sleep(1)
        except KeyboardInterrupt:
            print("\nMonitor detenido.")
