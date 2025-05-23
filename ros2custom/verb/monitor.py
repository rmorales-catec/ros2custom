import argparse
import subprocess
import threading
import re
import time
import os
from ros2cli.verb import VerbExtension
from tabulate import tabulate

class MonitorVerb(VerbExtension):
    """Monitorea un tópico en tiempo real con HZ, Delay y BW."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        parser.add_argument(
            'topic',
            help='Nombre del tópico a monitorear'
        )

    def main(self, *, args):
        topic = args.topic

        # Datos compartidos
        data = {
            'hz': '-', 'hz_win': '-',
            'delay': '-', 'delay_win': '-',
            'bw': '-', 'msg_size': '-'
        }

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
            # Línea con ancho de banda
            m = re.search(r'([0-9.]+)\s*(B|KB|MB)/s\s+from', line)
            if m:
                bw = float(m.group(1))
                unit = m.group(2).upper()
                if unit == 'B':
                    bw = bw / (1024.0 * 1024.0)
                elif unit == 'KB':
                    bw = bw / 1024.0
                # MB ya está en MB, no se convierte
                data['bw'] = round(bw, 2)
            # Línea con tamaño medio del mensaje
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
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            for line in proc.stdout:
                parser_fn(line)

        # Lanzar los monitores en hilos
        threading.Thread(target=launch_monitor, args=(['ros2', 'topic', 'hz', topic], parse_hz), daemon=True).start()
        threading.Thread(target=launch_monitor, args=(['ros2', 'topic', 'delay', topic], parse_delay), daemon=True).start()
        threading.Thread(target=launch_monitor, args=(['ros2', 'topic', 'bw', topic], parse_bw), daemon=True).start()

        try:
            while True:
                os.system('clear')  # para Linux/macOS, usa 'cls' en Windows
                table = [
                    ["Valor", f"{data['hz']} hz", f"{data['delay']} s", f"{data['bw']} MB/s"],
                    ["Info", f"win: {data['hz_win']}", f"win: {data['delay_win']}", f"msg_size: {data['msg_size']} MB"],
                ]


                headers = [' ', 'HZ', 'Delay', 'BW']
                print(tabulate(table, headers=headers, tablefmt='fancy_grid'))
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nMonitor detenido.")
