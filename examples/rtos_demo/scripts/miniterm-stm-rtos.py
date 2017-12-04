#
#	An extension of miniterm to start the serial console
#
#

import os
import sys
import codecs
import miniterm
import serial

def key_description(character):
    """generate a readable description for a key"""
    ascii_code = ord(character)
    if ascii_code < 32:
        return 'Ctrl+{:c}'.format(ord('@') + ascii_code)
    else:
        return repr(character)

def main(default_port=None, default_baudrate=9600, start_command=None):

    import argparse

    parser = argparse.ArgumentParser(
        description="Miniterm-stm-rtos - A simple terminal program for the serial console.")

    parser.add_argument(
        "port",
        nargs='?',
        help="serial port name ('-' to show port list)",
        default=default_port)

    parser.add_argument(
        "baudrate",
        nargs='?',
        type=int,
        help="set baud rate, default: %(default)s",
        default=default_baudrate)

    parser.add_argument(
        "--startcmd",
        help="set command sent on startup",
        dest='start_command',
        default=start_command)

    group = parser.add_argument_group("port settings")

    group.add_argument(
        "--parity",
        choices=['N', 'E', 'O', 'S', 'M'],
        type=lambda c: c.upper(),
        help="set parity, one of {N E O S M}, default: N",
        default='N')

    group.add_argument(
        "--rtscts",
        action="store_true",
        help="enable RTS/CTS flow control (default off)",
        default=False)

    group.add_argument(
        "--xonxoff",
        action="store_true",
        help="enable software flow control (default off)",
        default=False)

    group.add_argument(
        "--rts",
        type=int,
        help="set initial RTS line state (possible values: 0, 1)",
        default=None)

    group.add_argument(
        "--dtr",
        type=int,
        help="set initial DTR line state (possible values: 0, 1)",
        default=None)

    group.add_argument(
        "--ask",
        action="store_true",
        help="ask again for port when open fails",
        default=False)

    group = parser.add_argument_group("data handling")

    group.add_argument(
        "-e", "--echo",
        action="store_true",
        help="enable local echo (default off)",
        default=False)

    group.add_argument(
        "-f", "--filter",
        action="append",
        metavar="NAME",
        help="add text transformation",
        default=[])

    group.add_argument(
        "--eol",
        choices=['CR', 'LF', 'CRLF'],
        type=lambda c: c.upper(),
        help="end of line mode",
        default='CRLF')

    group = parser.add_argument_group("hotkeys")

    group = parser.add_argument_group("diagnostics")

    group.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="suppress non-error messages",
        default=False)

    group.add_argument(
        "--develop",
        action="store_true",
        help="show Python traceback on error",
        default=False)

    args = parser.parse_args()

    if args.filter:
        if 'help' in args.filter:
            sys.stderr.write('Available filters:\n')
            sys.stderr.write('\n'.join(
                '{:<10} = {.__doc__}'.format(k, v)
                for k, v in sorted(TRANSFORMATIONS.items())))
            sys.stderr.write('\n')
            sys.exit(1)
        filters = args.filter
    else:
        filters = ['default']
    while True:
        # no port given on command line -> ask user now
        if args.port is None or args.port == '-':
            try:
                args.port = ask_for_port()
            except KeyboardInterrupt:
                sys.stderr.write('\n')
                parser.error('user aborted and port is not given')
            else:
                if not args.port:
                    parser.error('port is not given')
        try:
            serial_instance = serial.serial_for_url(
                args.port,
                args.baudrate,
                parity=args.parity,
                rtscts=args.rtscts,
                xonxoff=args.xonxoff,
                do_not_open=True)

            if not hasattr(serial_instance, 'cancel_read'):
                # enable timeout for alive flag polling if cancel_read is not available
                serial_instance.timeout = 1

            if args.dtr is not None:
                if not args.quiet:
                    sys.stderr.write('--- forcing DTR {}\n'.format('active' if args.dtr else 'inactive'))
                serial_instance.dtr = args.dtr
            if args.rts is not None:
                if not args.quiet:
                    sys.stderr.write('--- forcing RTS {}\n'.format('active' if args.rts else 'inactive'))
                serial_instance.rts = args.rts

            serial_instance.open()
        except serial.SerialException as e:
            sys.stderr.write('could not open port {!r}: {}\n'.format(args.port, e))
            if args.develop:
                raise
            if not args.ask:
                sys.exit(1)
            else:
                args.port = '-'
        else:
            break

    term = miniterm.Miniterm(
        serial_instance,
        echo=args.echo,
        eol=args.eol.lower(),
        filters=filters)
    term.exit_character = chr(0x1d)  # GS/CTRL+]
    term.menu_character = chr(0x14)  # Menu: CTRL+T
    term.raw = False
    term.set_rx_encoding('UTF-8')
    term.set_tx_encoding('UTF-8')

    if not args.quiet:
        sys.stderr.write('--- Miniterm on {p.name}  {p.baudrate},{p.bytesize},{p.parity},{p.stopbits} ---\n'.format(
            p=term.serial))
        sys.stderr.write('--- Quit: {} | Menu: {} | Help: {} followed by {} ---\n'.format(
            key_description(term.exit_character),
            key_description(term.menu_character),
            key_description(term.menu_character),
            key_description('\x08')))

    term.start()
    # start_command = 's\r\n'
    if (args.start_command is not None):
        term.serial.write(term.tx_encoder.encode(args.start_command))
    try:
        term.join(True)
    except KeyboardInterrupt:
        pass
    if not args.quiet:
        sys.stderr.write("\n--- exit ---\n")
    term.join()
    term.close()

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if __name__ == '__main__':
    main()