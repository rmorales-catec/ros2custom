from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension
from ros2cli.verb import get_verb_extensions

class CustomCommand(CommandExtension):
    """ros2cli custom extension."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        parser.add_argument(
            '--include-hidden-topics', action='store_true',
            help='Consider hidden topics as well')
        
        # add arguments and sub-commands of verbs
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'ros2custom.verb', required=False)
        

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            self._subparser.print_help()
            return 0

        extension = getattr(args, '_verb')
        return extension.main(args=args)
