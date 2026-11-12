#!/usr/bin/env python3

import argparse
import functools
import http.server
from pathlib import Path
import socket
import threading


DISCOVERY_REQUEST = b"APP_OTA_DISCOVER_V1"
DISCOVERY_RESPONSE = b"APP_OTA_SERVER_V1"


def discovery_server(port: int, stop_event: threading.Event) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", port))
        server.settimeout(0.5)

        while not stop_event.is_set():
            try:
                request, client = server.recvfrom(64)
            except socket.timeout:
                continue

            if request == DISCOVERY_REQUEST:
                server.sendto(DISCOVERY_RESPONSE, client)
                print(f"Discovered by ESP32 at {client[0]}")


def parse_arguments() -> argparse.Namespace:
    default_directory = (
        Path(__file__).resolve().parents[1] / "platforms" / "esp32" / "build"
    )
    parser = argparse.ArgumentParser(
        description="Serve an ESP-IDF image and answer OTA discovery requests."
    )
    parser.add_argument(
        "--directory",
        type=Path,
        default=default_directory,
        help="directory containing app_esp32.bin",
    )
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--discovery-port", type=int, default=3232)
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    directory = arguments.directory.expanduser().resolve()
    firmware = directory / "app_esp32.bin"

    if not firmware.is_file():
        raise SystemExit(f"Firmware image not found: {firmware}")

    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler,
        directory=str(directory),
    )
    http_server = http.server.ThreadingHTTPServer(
        ("0.0.0.0", arguments.http_port),
        handler,
    )
    stop_event = threading.Event()
    discovery_thread = threading.Thread(
        target=discovery_server,
        args=(arguments.discovery_port, stop_event),
        daemon=True,
    )
    discovery_thread.start()

    print(f"Serving {firmware}")
    print(f"HTTP port: {arguments.http_port}")
    print(f"Discovery UDP port: {arguments.discovery_port}")

    try:
        http_server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        http_server.server_close()
        discovery_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
