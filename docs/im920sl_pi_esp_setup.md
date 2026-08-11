# IM920sL Raspberry Pi ↔ ESP32 通信設定手順

## 概要

Raspberry Pi 側の IM920sL を親機、ESP32 側の IM920sL を子機として設定し、
同一グループに登録したうえで双方向通信を確認する手順をまとめる。

今回使用した設定値は以下。

| 項目 | Raspberry Pi | ESP32 |
|---|---:|---:|
| ノード番号 | `0001` | `0002` |
| グループ番号 | `0001C07B` | 登録後 `0001C07B` |
| 通信チャンネル | `01` | `01` |
| 無線通信モード | `1` | `1` |
| ネットワークモード | `1` | `1` |
| グループ登録パスワード | `6B74630000000000` | `6B74630000000000` |

> `6B74630000000000` は、今回「ktc」を識別しやすい値として使うために、
> ASCII の `k=6B`, `t=74`, `c=63` に 0 埋めした 8 バイト値。
>
> IM920sL の `STRP` は ASCII 文字列ではなく、**16進数16桁（8バイト）**を指定する必要がある。

---

## 1. 前提

IM920sL 同士で通信するには、少なくとも以下を一致させる必要がある。

- グループ番号
- 通信チャンネル
- 無線通信モード
- ネットワークモード

また、同一グループ内ではノード番号を重複させない。

今回の構成では、

```text
Raspberry Pi
  IM920sL
  Node: 0001
  Group: 0001C07B
       │
       │ 920MHz RF
       │
ESP32
  IM920sL
  Node: 0002
  Group: 0001C07B
```

とする。

---

## 2. Raspberry Pi 側を親機として設定

Raspberry Pi 側の IM920sL はノード番号 `0001` にする。

IM920sL ではノード番号 `0001` を設定したモジュールが親機となり、
そのモジュール自身の固有 ID がグループ番号として使用される。

今回の Raspberry Pi 側 IM920sL は、

```text
RDID -> 0001C07B
```

だったため、親機のグループ番号は、

```text
0001C07B
```

となる。

### 設定コマンド

```text
ENWR
STNN 0001
STCH 01
STRT 1
STNM 1
STRP 6B74630000000000
```

期待するレスポンス:

```text
OK
OK
OK
OK
OK
OK
```

### Raspberry Pi からまとめて設定する Python

`UnagiFestival` の IM920 HAT 用コードを利用する場合:

```bash
uv run python - <<'PY'
import time

from unagifestival.tools.ps_controller import im_wireless as imw
from unagifestival.tools.ps_controller.config import SLAVE_ADR

im920 = imw.IMWireClass(SLAVE_ADR)


def wait_response(timeout: float = 1.0) -> str:
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        value = im920.Read_920()

        if value:
            return value.strip()

        time.sleep(0.01)

    return "<NO RESPONSE>"


try:
    time.sleep(0.5)

    while True:
        value = im920.Read_920()
        if not value:
            break
        print("startup:", repr(value))

    commands = (
        "ENWR",
        "STNN 0001",
        "STCH 01",
        "STRT 1",
        "STNM 1",
        "STRP 6B74630000000000",
    )

    for command in commands:
        display = (
            "STRP ****************"
            if command.startswith("STRP ")
            else command
        )

        print(f"TX: {display}")

        im920.Write_920(command)
        response = wait_response()

        print(f"RX: {response}")

        if response != "OK":
            print(f"ERROR: {command.split()[0]} failed")
            break

finally:
    im920.gpio_clean()
PY
```

---

## 3. Raspberry Pi 側の設定確認

```text
RDID
RDNN
RDGN
RDCH
RDRT
RDNM
```

期待値:

```text
RDID -> 0001C07B
RDNN -> 0001
RDGN -> 0001C07B
RDCH -> 01
RDRT -> 1
RDNM -> 1
```

---

## 4. ESP32 側を子機として設定

```text
ENWR
STNN 0002
STCH 01
STRT 1
STNM 1
```

確認:

```text
RDNN
RDCH
RDRT
RDNM
```

期待値:

```text
RDNN -> 0002
RDCH -> 01
RDRT -> 1
RDNM -> 1
```

---

## 5. ESP32 側に登録先グループを設定

Pi 側のグループ番号 `0001C07B` をターゲットとして設定する。

```text
ENWR
STTG 0001C07B
```

確認:

```text
RDTG
```

期待値:

```text
0001C07B
```

---

## 6. ESP32 側に同じ登録パスワードを設定

```text
STRP 6B74630000000000
```

期待値:

```text
OK
```

### 注意

以下は無効。

```text
STRP ktc
```

`STRP` は **16進数16桁固定**のため、文字列 `ktc` をそのまま渡すと `NG` になる。

---

## 7. ESP32 からグループ登録を開始

```text
ENWR
STGP
```

`STGP -> OK` は登録成功ではなく、**登録処理を開始できた**ことを表す。

処理開始:

```text
REGSTART
```

登録成功:

```text
GRNOREGD
```

登録失敗:

```text
REGERROR
```

登録成功後に確認する。

```text
RDGN
```

期待値:

```text
0001C07B
```

---

## 8. スニファモードを解除する

デバッグ中に ESP 側で `ESNF` を使用していた場合、そのままでは `TXDA` を実行できない。

スニファモード中:

```text
TXDA 414243
NG
```

通常通信へ戻す:

```text
ENWR
DSNF
```

期待値:

```text
OK
OK
```

`ENWR` 状態で `DSNF` を実行すると Flash にも保存される。

---

## 9. Pi → ESP の疎通確認

Pi 側:

```text
TXDA 313233
```

`31 32 33` は ASCII で `123`。

Pi 側で `OK` が返れば送信処理自体は完了しているが、受信側への到達保証ではないため ESP 側でも確認する。

通常モードでは概ね:

```text
00,0001,xx:31,32,33
```

スニファモード時に実際に確認できた例:

```text
Tm002A7752,Gn0001C07B,Tx0001,RxFFFF,Id0001C07B,RsD2
Fm0001,To0000,Mi4C54,Sl0400,Tt0A,Hp00
Rt 0001
Dt 31,32,33
```

ここで、

- `Gn0001C07B`
- `Tx0001`
- `Dt 31,32,33`

を確認できれば Pi → ESP の RF 通信は成立している。

---

## 10. ESP → Pi の疎通確認

ESP 側:

```text
TXDA 414243
```

`41 42 43` は ASCII で `ABC`。

期待:

```text
CMD -> TXDA 414243
IM920 RAW <- OK
```

Pi 側では概ね:

```text
00,0002,xx:41,42,43
```

を確認する。

Pi → ESP と ESP → Pi の両方が確認できれば、IM920sL の双方向 RF 通信は成立している。

---

## 11. 実際の制御パケットを送信

疎通確認後、`UnagiFestival` の制御パケットを送る。

例:

```text
TXDA 4304A80000
```

内容:

```text
43 04 A8 00 00
```

意味:

```text
43    CONTROL
04    DRIVE
A8    vx = -88
00    vy = 0
00    wz = 0
```

ESP 側で CONTROL / DRIVE としてデコードされれば、
RF 通信だけでなくアプリケーションの通信プロトコルまで正常に通っている。

---

## 12. 最終的な正常設定

### Raspberry Pi

```text
RDID -> 0001C07B
RDNN -> 0001
RDGN -> 0001C07B
RDCH -> 01
RDRT -> 1
RDNM -> 1
```

登録パスワード:

```text
6B74630000000000
```

### ESP32

```text
RDNN -> 0002
RDGN -> 0001C07B
RDCH -> 01
RDRT -> 1
RDNM -> 1
```

さらに通常通信時は `DSNF` 状態にする。

---

## トラブルシューティング

### `STRP ktc` が `NG`

`STRP` は文字列ではなく16進数16桁を要求する。

今回:

```text
STRP 6B74630000000000
```

を使用する。

### `TXDA` が `NG`

まず:

```text
RDGN
```

が `0001C07B` になっているか確認する。

次にスニファモードを確認する。

```text
ENWR
DSNF
```

を実行して通常モードへ戻す。

### `TXDA -> OK` なのに相手で受信できない

`TXDA` の `OK` は受信成功を意味しない。

両側で以下を確認する。

```text
RDGN
RDCH
RDRT
RDNM
```

今回の正常値:

```text
GN = 0001C07B
CH = 01
RT = 1
NM = 1
```

---

## 参考資料

- Interplan `IM920sL 取扱説明書 Rev.1.5`
  - グループ番号の登録・消去
  - `STRP`
  - `STTG`
  - `STGP`
  - `ESNF` / `DSNF`
  - `TXDA`
- Interplan `IM920sL クイックスタートガイド`
