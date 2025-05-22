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
            if "Average bandwidth" in line:
                m = re.search(r'Average bandwidth:\s+([\d.]+\s+\w+/s)', line)
                if m:
                    data['bw'] = m.group(1)
            elif "Average message size" in line:
                m = re.search(r'Average message size:\s+([\d.]+\s+\w+)', line)
                if m:
                    data['msg_size'] = m.group(1)

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
                    ['Valor', data['hz'], data['delay'], data['bw']],
                    ['Info', f'win: {data["hz_win"]}', f'win: {data["delay_win"]}', data['msg_size']],
                ]
                headers = [' ', 'HZ', 'Delay', 'BW']
                print(tabulate(table, headers=headers, tablefmt='fancy_grid'))
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nMonitor detenido.")
