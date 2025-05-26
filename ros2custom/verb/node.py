from ros2cli.command import CommandExtension
from ros2cli.verb import VerbExtension
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, ListParameters
import threading
import time
from tabulate import tabulate
import subprocess

class NodeVerb(VerbExtension):
    """Monitor real-time parameters, services, actions, and pub/sub of a given node."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'node_name',
            help='Nombre completo del nodo (e.g. /my_node)')
        parser.add_argument(
            '-serv', '--services',
            action='store_true',
            help='Mostrar servicios del nodo')
        parser.add_argument(
            '-act', '--actions',
            action='store_true',
            help='Mostrar acciones del nodo')
        parser.add_argument(
            '-pubsub', '--pubsub',
            action='store_true',
            help='Mostrar tabla de publishers y suscriptores del nodo')
        return parser

    def main(self, *, args):
        rclpy.init()
        monitor = NodeMonitor(
            args.node_name,
            show_services=args.services,
            show_actions=args.actions,
            show_pubsub=args.pubsub
        )
        try:
            monitor.spin()
        except KeyboardInterrupt:
            print('\nCancelado por el usuario.')
        finally:
            monitor.destroy_node()
            rclpy.shutdown()

class NodeMonitor(Node):
    def __init__(self, node_name, show_services=False, show_actions=False, show_pubsub=False):
        super().__init__('param_monitor')
        self.target_node = node_name
        self.show_services = show_services
        self.show_actions = show_actions
        self.show_pubsub = show_pubsub

        self.parameters = {}
        self.current_services = set()
        self.current_actions = set()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self.update_rate = 1.0

    def spin(self):
        print(f'Consultando nodo {self.target_node}...')
        self.update_parameters()
        if self.show_services:
            self.update_services()
        if self.show_actions:
            self.update_actions()
        if self.show_pubsub:
            self.update_pubsub()

    def update_services(self):
        node_names_and_namespaces = self.get_node_names_and_namespaces_with_enclaves()
        found = False
        for name, ns, _ in node_names_and_namespaces:
            full_name = ns.rstrip('/') + '/' + name if ns != '/' else '/' + name
            if full_name == self.target_node:
                found = True
                srv_list = self.get_service_names_and_types_by_node(name, ns)
                new_services = set()
                for srv_name, srv_types in srv_list:
                    # Filtrar para excluir servicios que sean acciones
                    if '/_action/' not in srv_name:
                        new_services.add((srv_name, tuple(srv_types)))
                if new_services != self.current_services:
                    self.current_services = new_services
                    self.print_services()
                break
        if not found:
            print(f'[WARN] Nodo {self.target_node} no encontrado en esta iteración.')
    
    def print_services(self):
        # Prepara lista para tabulate [[nombre, tipos], ...]
        table = [[name, ", ".join(types)] for name, types in sorted(self.current_services)]
        print('\n[Servicios disponibles]')
        print(tabulate(table, headers=['Nombre', 'Tipo(s)'], tablefmt='fancy_grid'))

    def update_actions(self):
        try:
            # Ejecuta 'ros2 action list' y captura la salida
            result = subprocess.run(['ros2', 'action', 'list'], capture_output=True, text=True, check=True)
            all_actions = result.stdout.strip().split('\n')

            if all_actions and all_actions[0] != '':
                print('\n[Acciones encontradas]')
                for action in sorted(all_actions):
                    print(f'- {action}')
            else:
                print('\n[Acciones] No se encontraron acciones activas.')
        except subprocess.CalledProcessError as e:
            print(f'Error al ejecutar ros2 action list: {e}')


    def print_actions(self):
        print('\n[Acciones disponibles]')
        for name, types in sorted(self.current_actions):
            print(f'- {name}: {", ".join(types)}')

    def update_pubsub(self):
        node_names_and_namespaces = self.get_node_names_and_namespaces_with_enclaves()
        found = False
        for name, ns, _ in node_names_and_namespaces:
            full_name = ns.rstrip('/') + '/' + name if ns != '/' else '/' + name
            if full_name == self.target_node:
                found = True
                publishers = self.get_publisher_names_and_types_by_node(name, ns)
                subscribers = self.get_subscriber_names_and_types_by_node(name, ns)

                pub_topics = sorted([p[0] for p in publishers])
                sub_topics = sorted([s[0] for s in subscribers])

                max_len = max(len(pub_topics), len(sub_topics))
                # Construimos la lista de filas para tabulate
                table_data = []
                for i in range(max_len):
                    sub_topic = sub_topics[i] if i < len(sub_topics) else ''
                    pub_topic = pub_topics[i] if i < len(pub_topics) else ''
                    table_data.append([sub_topic, pub_topic])

                print('\n[Topics]')
                print(tabulate(table_data, headers=["Suscriptores", "Publishers"], tablefmt="fancy_grid"))
                break
        if not found:
            print(f'[WARN] Nodo {self.target_node} no encontrado para pub/sub.')

    def update_parameters(self):
        client = self.create_client(ListParameters, f'{self.target_node}/list_parameters')
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=1.0)
        req = ListParameters.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            names = future.result().result.names
            self.get_and_print_parameters(names)
        else:
            self.get_logger().error(f'Error al obtener parámetros de {self.target_node}')

    def get_and_print_parameters(self, param_names):
        client = self.create_client(GetParameters, f'{self.target_node}/get_parameters')
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=1.0)

        req = GetParameters.Request()
        req.names = param_names
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            values = future.result().values
            with self._lock:
                changed = False
                for name, val in zip(param_names, values):
                    str_val = self._to_string(val)
                    if self.parameters.get(name) != str_val:
                        self.parameters[name] = str_val
                        changed = True
                if changed:
                    self.print_parameters()
        else:
            self.get_logger().error('No se pudieron obtener los valores de parámetros.')

    def _to_string(self, param_value):
        from rcl_interfaces.msg import ParameterType
        type_ = param_value.type
        if type_ == ParameterType.PARAMETER_STRING:
            return param_value.string_value
        elif type_ == ParameterType.PARAMETER_INTEGER:
            return str(param_value.integer_value)
        elif type_ == ParameterType.PARAMETER_DOUBLE:
            return str(param_value.double_value)
        elif type_ == ParameterType.PARAMETER_BOOL:
            return str(param_value.bool_value)
        elif type_ == ParameterType.PARAMETER_STRING_ARRAY:
            return str(param_value.string_array_value)
        elif type_ == ParameterType.PARAMETER_INTEGER_ARRAY:
            return str(param_value.integer_array_value)
        elif type_ == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return str(param_value.double_array_value)
        elif type_ == ParameterType.PARAMETER_BOOL_ARRAY:
            return str(param_value.bool_array_value)
        else:
            return '<desconocido>'

    def print_parameters(self):
            # Prepara una lista de listas para tabulate [[nombre, valor], ...]
            table = [[name, val] for name, val in self.parameters.items()]
            print('\n[Parámetros actualizados]')
            print(tabulate(table, headers=['Nombre', 'Valor'], tablefmt='fancy_grid'))
