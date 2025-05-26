from ros2cli.command import CommandExtension
from ros2cli.verb import VerbExtension
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters, ListParameters
import threading


class NodeVerb(VerbExtension):
    """Monitoriza parámetros y servicios de un nodo (una sola ejecución)."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'node_name',
            help='Nombre completo del nodo (e.g. /my_node)')
        parser.add_argument(
            '-serv', '--services',
            action='store_true',
            help='Mostrar servicios del nodo')
        return parser

    def main(self, *, args):
        rclpy.init()
        monitor = NodeMonitor(args.node_name, show_services=args.services)
        try:
            monitor.spin_once()
        except KeyboardInterrupt:
            print('\nCancelado por el usuario.')
        finally:
            monitor.destroy_node()
            rclpy.shutdown()


class NodeMonitor(Node):
    def __init__(self, node_name, show_services=False):
        super().__init__('param_monitor')
        self.target_node = node_name
        self.show_services = show_services
        self.parameters = {}
        self.current_services = set()
        self._lock = threading.Lock()

    def spin_once(self):
        print(f'Consultando nodo {self.target_node}...')
        self.update_parameters()
        if self.show_services:
            self.update_services()

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
                    new_services.add((srv_name, tuple(srv_types)))
                self.current_services = new_services
                self.print_services()
                break
        if not found:
            print(f'[WARN] Nodo {self.target_node} no encontrado.')

    def print_services(self):
        print('\n[Servicios disponibles]')
        for name, types in sorted(self.current_services):
            print(f'- {name}: {", ".join(types)}')

    def update_parameters(self):
        client = self.create_client(ListParameters, f'{self.target_node}/list_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Servicio no disponible: {client.srv_name}')
            return

        req = ListParameters.Request()
        future = client.call_async(req)

        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(future, timeout_sec=2.0)
        executor.remove_node(self)

        if future.done() and future.result():
            names = future.result().result.names
            self.get_and_print_parameters(names)
        else:
            self.get_logger().error(f'Error al obtener parámetros de {self.target_node}')

    def get_and_print_parameters(self, param_names):
        client = self.create_client(GetParameters, f'{self.target_node}/get_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Servicio no disponible: {client.srv_name}')
            return

        req = GetParameters.Request()
        req.names = param_names
        future = client.call_async(req)

        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(future, timeout_sec=2.0)
        executor.remove_node(self)

        if future.done() and future.result():
            values = future.result().values
            with self._lock:
                for name, val in zip(param_names, values):
                    self.parameters[name] = self._to_string(val)
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
        print('\n[Parámetros]')
        for name, val in self.parameters.items():
            print(f'- {name}: {val}')
