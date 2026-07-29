# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Small bounded read-only transports shared by bridge nodes."""
import socket


class UdpReceiveTransport:  # noqa: D101
    def __init__(self, address: str, port: int, allowed_remote_ip: str = "") -> None:  # noqa: D107
        self.allowed_remote_ip = allowed_remote_ip
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind((address, port))

    def read(self) -> tuple[bytes, tuple[str, int]] | None:  # noqa: D102
        try:
            data, remote = self.socket.recvfrom(4096)
        except BlockingIOError:
            return None
        if self.allowed_remote_ip and remote[0] != self.allowed_remote_ip:
            return None
        return data, remote

    def close(self) -> None:  # noqa: D102
        self.socket.close()


class BoundedLineBuffer:  # noqa: D101
    def __init__(self, maximum: int = 256) -> None:  # noqa: D107
        self.maximum = maximum
        self.buffer = bytearray()
        self.dropped = 0

    def feed(self, data: bytes) -> list[bytes]:  # noqa: D102
        lines = []
        for byte in data:
            if byte == 10:
                lines.append(bytes(self.buffer).rstrip(b"\r"))
                self.buffer.clear()
            elif len(self.buffer) < self.maximum:
                self.buffer.append(byte)
            else:
                self.buffer.clear()
                self.dropped += 1
        return lines
