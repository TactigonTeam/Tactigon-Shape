#********************************************************************************
# Copyright (c) 2025 Next Industries s.r.l.
#
# This program and the accompanying materials are made available under the
# terms of the Apache 2.0 which is available at http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0
#
# Project Name:
# Tactigon Soul - Shape
# 
# Release date: 30/09/2025
# Release version: 1.0
#
# Contributors:
# - Massimiliano Bellino
# - Stefano Barbareschi
#********************************************************************************/

import logging
import argparse
import signal
from tactigon_shapes import TactigonShapes

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

def run():
    parser = argparse.ArgumentParser("Tactigon Shapes")
    parser.add_argument("-A", "--address", help="Server address", type=str, default="0.0.0.0")
    parser.add_argument("-P", "--port", help="Server port", type=int, default=5123)
    args = parser.parse_args()

    server = TactigonShapes(args.address.strip(), args.port, True)

    signal.signal(signal.SIGTERM, lambda s, h: server.stop())
    signal.signal(signal.SIGINT, lambda s, h: server.stop())

    try:
        server.serve()
    except KeyboardInterrupt:
        server.stop()

if __name__ == "__main__":
    run()