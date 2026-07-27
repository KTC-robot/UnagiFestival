"""IM920sL 受信側サンプル.

受信したデータを表示し、"OK: <data>" の形式で送り返す (オウム返し)。

Usage:
    python src/920hz/rx.py
    python src/920hz/rx.py --port /dev/ttyUSB0 --baudrate 19200
"""

from __future__ import annotations

import argparse
import sys

from im920sl import IM920sL


def main() -> None:
    parser = argparse.ArgumentParser(description="IM920sL 受信側サンプル")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=19200)
    args = parser.parse_args()

    print(f"IM920sL RX: port={args.port} baudrate={args.baudrate}")  # noqa: T201
    print("受信待機中... Ctrl+C で終了")  # noqa: T201

    try:
        with IM920sL(args.port, args.baudrate) as module:
            while True:
                result = module.recv()
                if result is None:
                    continue
                node_id, rssi, data = result
                print(f"[RX] node={node_id} rssi={rssi} data={data!r}")  # noqa: T201
                reply = f"OK: {data}"
                """
                レスポンスは帯域の無駄使い
                module.send(reply)
                print(f"[TX] {reply!r}")  # noqa: T201
                """
    except KeyboardInterrupt:
        pass
    finally:
        print("\n終了")  # noqa: T201
        sys.exit(0)


if __name__ == "__main__":
    main()
