#!/usr/bin/env python3

import argparse
import ast
import functools
import http.server
from pathlib import Path
import re
import socket
import threading


CONFIG_HEADER = (
    Path(__file__).resolve().parents[1]
    / "platforms"
    / "esp32"
    / "main"
    / "config"
    / "config_ota.h"
)


def read_config() -> dict[str, str]:
    defines: dict[str, str] = {}
    pattern = re.compile(r"^\s*#define\s+(APP_OTA_\w+)\s+(.+?)\s*$")

    for line in CONFIG_HEADER.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            defines[match.group(1)] = match.group(2)

    return defines


def config_integer(defines: dict[str, str], name: str) -> int:
    return int(defines[name].rstrip("uUlL"), 0)


def config_string(defines: dict[str, str], name: str) -> str:
    value = ast.literal_eval(defines[name])

    if not isinstance(value, str):
        raise ValueError(f"{name} is not a string")

    return value


def discovery_server(
    port: int,
    expected_request: bytes,
    response: bytes,
    stop_event: threading.Event,
) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", port))
        server.settimeout(0.5)

        while not stop_event.is_set():
            try:
                received_request, client = server.recvfrom(64)
            except socket.timeout:
                continue

            if received_request == expected_request:
                server.sendto(response, client)
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
        help="ESP-IDF build directory",
    )
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    defines = read_config()
    http_port = config_integer(defines, "APP_OTA_SERVER_PORT")
    discovery_port = config_integer(defines, "APP_OTA_DISCOVERY_PORT")
    firmware_path = config_string(defines, "APP_OTA_FIRMWARE_PATH")
    discovery_request = config_string(
        defines, "APP_OTA_DISCOVERY_REQUEST"
    ).encode()
    discovery_response = config_string(
        defines, "APP_OTA_DISCOVERY_RESPONSE"
    ).encode()
    directory = arguments.directory.expanduser().resolve()
    firmware = directory / firmware_path.lstrip("/")

    if not firmware.is_file():
        raise SystemExit(f"Firmware image not found: {firmware}")

    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler,
        directory=str(directory),
    )
    http_server = http.server.ThreadingHTTPServer(
        ("0.0.0.0", http_port),
        handler,
    )
    stop_event = threading.Event()
    discovery_thread = threading.Thread(
        target=discovery_server,
        args=(
            discovery_port,
            discovery_request,
            discovery_response,
            stop_event,
        ),
        daemon=True,
    )
    discovery_thread.start()

    print(f"Serving {firmware}")
    print(f"HTTP port: {http_port}")
    print(f"Discovery UDP port: {discovery_port}")

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
