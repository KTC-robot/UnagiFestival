# APモード下での完全オフラインuv環境構築手順

Raspberry Pi 5（aarch64）がAPモードで稼働し、外部ネットワークに接続できない環境において、ホストPC側で依存関係を解決してオフライン構築する手順。

## 1. requirements.txtの出力 (ホストPC)
プロジェクトの `pyproject.toml` から、依存パッケージのリストをコンパイルする。

```zsh
uv pip compile pyproject.toml -o requirements.txt
```

## 2. パッケージのダウンロード (ホストPC)

Raspberry Pi 5のアーキテクチャ（aarch64, Python 3.12）を指定し、コンパイル済みバイナリ（Wheel）のみを一括ダウンロードする。

```zsh
pip download -r requirements.txt setuptools wheel -d ./wheels \
  --platform manylinux2014_aarch64 \
  --python-version 312 \
  --implementation cp \
  --abi cp312 \
  --only-binary=:all:
```

## 3. ダウンロードエラー時の対応 (ホストPC)

`evdev` や `rplidar-roboticia` など、aarch64用のコンパイル済みWheelがPyPIに存在しないパッケージは上記コマンドでエラー（`No matching distribution found`）となる。

該当パッケージはソースコード（`.tar.gz`）として個別にダウンロードする。

```zsh
# エラーになったパッケージをソース指定で取得
pip download evdev==1.9.3 rplidar-roboticia==0.9.5 --no-binary=:all: -d ./wheels
```

※ 取得後、`requirements.txt` から上記パッケージの記述を削除し、再度「2. パッケージのダウンロード」を実行して残りのWheelを収集すること。

## 4. Raspberry Piへの送信 (ホストPC)

収集した `wheels` ディレクトリと `requirements.txt` を、Raspberry Piのプロジェクトディレクトリへ転送する。

```zsh
scp -r ./wheels requirements.txt pi@<Raspberry_PiのIPアドレス>:~/test_tool/
```

## 5. オフラインインストール (Raspberry Pi側)

ネットワーク通信を遮断し、ローカルの `wheels` ディレクトリ（`--find-links`）のみを参照してインストールを実行する。

```zsh
cd ~/test_tool

# 仮想環境の作成
uv venv

# 1. ビルドツールを先行インストール
uv pip install --no-index --find-links=./wheels setuptools wheel

# 2. 依存パッケージのインストール (ソースコードが含まれる場合はここでコンパイルが走る)
uv pip install --no-index --find-links=./wheels -r requirements.txt

# 3. プロジェクト本体のインストール (ビルド分離と依存解決を無効化)
uv pip install --no-index --find-links=./wheels --no-build-isolation --no-deps -e .
```

## 6. 実行 (Raspberry Pi側)

実行時は、`uv` がPyPIへ同期確認に行くのを防ぐため、必ず `--no-sync` フラグを付与する。

```zsh
uv run --no-sync python src/tools/cli/main.py --help
```
