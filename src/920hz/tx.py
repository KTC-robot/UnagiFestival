"""IM920sL 送信側サンプル.

キーボードから入力した文字列を無線送信し、
受信データをバックグラウンドスレッドで表示する。

Usage:
    python src/920hz/tx.py
    python src/920hz/tx.py --port /dev/ttyUSB0 --baudrate 19200
"""

from __future__ import annotations

import argparse
import sys
import threading

from im920sl import IM920sL


def _recv_loop(module: IM920sL, stop: threading.Event) -> None:
    while not stop.is_set():
        result = module.recv()
        if result is not None:
            node_id, rssi, data = result
            print(f"\r[RX] node={node_id} rssi={rssi} data={data!r}")  # noqa: T201
            print("送信> ", end="", flush=True)  # noqa: T201


def main() -> None:
    parser = argparse.ArgumentParser(description="IM920sL 送信側サンプル")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=19200)
    args = parser.parse_args()

    print(f"IM920sL TX: port={args.port} baudrate={args.baudrate}")  # noqa: T201
    print("Ctrl+C で終了 | 空行はスキップ")  # noqa: T201

    stop = threading.Event()

    try:
        with IM920sL(args.port, args.baudrate) as module:
            recv_thread = threading.Thread(target=_recv_loop, args=(module, stop), daemon=True)
            recv_thread.start()

            while True:
                try:
                    text = input("送信> ")
                except EOFError:
                    break
                if not text.strip():
                    continue
                module.send(text)
                print(f"[TX] {text!r}")  # noqa: T201

    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print("\n終了")  # noqa: T201
        sys.exit(0)


if __name__ == "__main__":
    main()
