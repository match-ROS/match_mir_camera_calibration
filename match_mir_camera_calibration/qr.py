"""Scannable phone links using the existing OpenCV dependency."""
import socket

import cv2
import numpy as np


def advertised_host(bind_host, observer):
    if bind_host != '0.0.0.0':
        return bind_host
    # UDP connect selects the local interface without sending any datagram.
    for destination in (observer, '192.0.2.1'):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
                connection.connect((destination, 9))
                return connection.getsockname()[0]
        except OSError:
            continue
    return socket.gethostname()


def qr_modules(url):
    if not hasattr(cv2, 'QRCodeEncoder_create'):
        raise ValueError('OpenCV mit QRCodeEncoder wird benötigt (Jazzy: python3-opencv).')
    modules = cv2.QRCodeEncoder_create().encode(url)
    # At least four white modules around all edges, regardless of encoder border.
    return np.pad(modules, 4, constant_values=255)


def qr_image(url):
    return cv2.resize(qr_modules(url), None, fx=10, fy=10, interpolation=cv2.INTER_NEAREST)


def terminal_qr(url):
    modules = qr_modules(url)
    if len(modules) % 2:
        modules = np.pad(modules, ((0, 1), (0, 0)), constant_values=255)
    blocks = {(False, False): ' ', (True, False): '▀',
              (False, True): '▄', (True, True): '█'}
    lines = []
    for row in range(0, len(modules), 2):
        line = ''.join(blocks[(bool(top < 128), bool(bottom < 128))]
                       for top, bottom in zip(modules[row], modules[row+1]))
        # Explicit black on white works with both light and dark terminals.
        lines.append('\033[30;47m'+line+'\033[0m')
    return '\n'.join(lines)
