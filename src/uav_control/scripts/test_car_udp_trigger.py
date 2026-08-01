#!/usr/bin/env python3
"""Send safe localhost-only protocol samples to a running gateway."""

import argparse
import socket

from uav_control.car_protocol import xor_checksum


def make_frame(payload):
    return f'${payload}*{xor_checksum(payload):02X}\r\n'.encode('ascii')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=19011)
    args = parser.parse_args()
    samples = (
        'EVT,0,BOOT',
        'CAR,0,0,0,0,00,0,0,0,00',
        'EVT,1,START',
        'EVT,1,START',
        'CAR,1,1,100,100,01,0,100,100,00',
        'PONG',
    )
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for sample in samples:
            sock.sendto(
                (sample.encode('ascii') if sample == 'PONG' else make_frame(sample)),
                ('127.0.0.1', args.port))


if __name__ == '__main__':
    main()
