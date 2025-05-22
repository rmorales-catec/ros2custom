from ros2cli.verb import VerbExtension

class PruebaVerb(VerbExtension):
    """Prints Hello World on the terminal."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--simple', '-s', action='store_true',
            help="Display the message in a simple form.")

    def main(self, *, args):
        message = "Hello, ROS 2 World!"
        if not args.simple:
            message = (
               "Hello, this is a custom command line tool "
            )
        print(message)
