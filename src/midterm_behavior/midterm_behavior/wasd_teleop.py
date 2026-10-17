"""Keyboard teleop with WASD keys: hold a key to drive, release to stop."""

import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

HELP = """
WASD teleop - hold a key to drive, release to stop
---------------------------------------------------
        w                 q / e : forward + turn left / right
    a   s   d       PgUp / PgDn : speed up / down (10 %)
                          space : stop
                          Ctrl+C: quit
"""

# key -> (linear direction, angular direction)
BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'q': (1.0, 1.0),
    'e': (1.0, -1.0),
    ' ': (0.0, 0.0),
}

# Escape sequences sent by the terminal
PAGE_UP = '\x1b[5~'
PAGE_DOWN = '\x1b[6~'


class WasdTeleop(Node):

    def __init__(self):
        super().__init__('wasd_teleop')
        self.linear_speed = self.declare_parameter('linear_speed', 0.3).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.8).value
        # Terminals send no key-release event: stop when no key arrives within this time.
        # Must be longer than the OS key-repeat delay (~0.5 s) to avoid stuttering.
        self.release_timeout = self.declare_parameter('release_timeout', 0.6).value
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish(self, linear_dir, angular_dir):
        msg = Twist()
        msg.linear.x = linear_dir * self.linear_speed
        msg.angular.z = angular_dir * self.angular_speed
        self.publisher.publish(msg)

    def print_status(self):
        print(f'\rlinear {self.linear_speed:.2f} m/s | angular {self.angular_speed:.2f} rad/s   ',
              end='', flush=True)


def read_key(timeout):
    """Return one key (a single char or a full escape sequence), or None on timeout."""
    fd = sys.stdin.fileno()
    if not select.select([fd], [], [], timeout)[0]:
        return None
    # Unbuffered reads: buffered stdin would hide the rest of an escape sequence from select()
    key = os.read(fd, 1).decode(errors='ignore')
    if key == '\x1b':
        # Special keys (PgUp, arrows, ...) arrive as ESC followed by a few chars
        while select.select([fd], [], [], 0.01)[0]:
            key += os.read(fd, 1).decode(errors='ignore')
            if key[-1].isalpha() or key[-1] == '~':
                break
    return key


def main():
    # Handle Ctrl+C ourselves so the context is still alive to send the final stop command
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WasdTeleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    node.print_status()
    moving = False
    try:
        tty.setcbreak(sys.stdin.fileno())
        # Disable signal keys: Ctrl+C arrives as '\x03' and is handled in the loop,
        # so shutdown is never interrupted before the stop command is sent
        attrs = termios.tcgetattr(sys.stdin)
        attrs[3] &= ~termios.ISIG
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, attrs)
        while rclpy.ok():
            key = read_key(node.release_timeout)
            if key is None:
                if moving:
                    node.publish(0.0, 0.0)
                    moving = False
                continue
            key = key.lower()
            if key == '\x03':
                break
            if key in BINDINGS:
                linear_dir, angular_dir = BINDINGS[key]
                node.publish(linear_dir, angular_dir)
                moving = key != ' '
            elif key == PAGE_UP:
                node.linear_speed *= 1.1
                node.angular_speed *= 1.1
                node.print_status()
            elif key == PAGE_DOWN:
                node.linear_speed *= 0.9
                node.angular_speed *= 0.9
                node.print_status()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            # DiffDrive keeps the last command: make sure the stop message goes out before exit
            node.publish(0.0, 0.0)
            time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
