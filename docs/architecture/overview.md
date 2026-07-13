# NSE2026 アーキテクチャ概要

この文書は、`README.md` とは分けて管理するリポジトリ構成・設計意図・単一機体の運用方針をまとめる。

- `README.md`: セットアップ手順、依存導入、実行コマンドの入口
- `docs/architecture/overview.md`: ファイル構成、責務、設定値とデバッグ運用の境界

本リポジトリは1機体専用である。機体名による設定切り替えや機体別ログ階層は持たない。

## ログ出力先早見表

| 実行入口 | 主な出力 | 既定の保存先 |
| --- | --- | --- |
| `python3 main.py` | ミッションCSV + 到達時PNG | `/home/pi/logs_nse2026/mission/` |
| `runs/orch/*.py` | ミッションCSV + 到達時PNG | `/home/pi/logs_nse2026/debug/<label>/` |
| `lib/roi_capture.py` | ROI参照PNG + 最新ROI | `/home/pi/logs_nse2026/roi/` |
| `runs/cam/detect_dbg.py` | `debug.csv` + 各種PNG + 到達時PNG | `/home/pi/logs_nse2026/camera_debug_<timestamp>_<ms>/` |
| `runs/cam/capture.py` | 撮影JPEG群 | `/home/pi/logs_nse2026/capture_<timestamp>[_session]/` |
| `runs/diag/*.py` | 端末出力 | 原則として保存ファイルなし |
| `runs/spec/*.py` | `unittest` 出力 | 保存ファイルなし |

- ミッションCSVは `robust_log_YYYY-mmdd-HHMMSS-uuuuuu.csv` 形式で保存する。
- CSV と `capture_reached.png` は同じログディレクトリへ保存する。
- `runs/orch` は `--log-dir` で保存先を明示できる。本番の `main.py` は設定上書き引数を持たない。

## ファイル構成

```text
├── README.md
├── main.py
├── mission.toml.example
├── analysis/
│   ├── log.py
│   └── explorer.py
├── docs/
├── gerber/
├── lib/
├── mission/
│   ├── const.py
│   ├── config.py
│   ├── ctrl.py
│   ├── gps_util.py
│   ├── motor_map.py
│   ├── nav.py
│   ├── run.py
│   ├── st.py
│   ├── mgr/
│   └── phases/
└── runs/
    ├── cam/
    ├── diag/
    ├── orch/
    ├── spec/
    └── telemetry/
```

## この構成の意図

- `main.py` は本番入口だけにする。
- 本番ロジックは `mission/` に置く。
- デバッグや個別試験は `runs/` に置き、本番コードを書き換えて試験しない。
- センサーや画像処理の低レイヤは `lib/` に置く。
- ログ解析は `analysis/` に分け、実機制御と切り離す。
- 設計判断や運用手順は `docs/` に集約する。

## `mission/` 内部の責務

- `const.py`: 単一機体のハードウェア設定、制御定数、閾値
- `config.py`: Git管理外の `mission.toml` の検証と読み込み
- `st.py`: スレッドセーフな共有状態
- `nav.py`: 距離・方位などの副作用が少ない計算
- `gps_util.py`: GPS入出力とNMEA処理
- `motor_map.py`: 論理的な走行方向とモータードライバ出力の対応
- `ctrl.py`: 初期化、ログ作成、フェーズディスパッチ、実行範囲制限
- `run.py`: `run_full_mission()`、`run_phase_sequence()`、`run_single_phase()` の入口
- `mgr/`: ハード初期化、センサー取得、モータ、LED、無線制御
- `phases/`: フェーズ共通インターフェースと `p0` から `p7` の状態遷移

通常遷移は `0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7` とし、一部フェーズは `4 -> 3` のようなフォールバックを持つ。

## 単一機体仕様

- 機体識別ファイルは使用しない。
- 機体名を起動引数や環境変数から受け取らない。
- モータ方向、PWM補正、GPIOなどの固定値は `mission/const.py` に置く。
- モータ設定は現在運用する実機の値に固定する。
- ログCSVに機体名列を出力しない。
- ログと解析結果を機体名で分類しない。

複数機体運用へ戻すことを前提とした抽象化は追加しない。将来ハードウェアを交換した場合は、現在運用する1機体の値として `mission/const.py` を更新し、診断と実機試験で確認する。

## 実行入口

- 本番実行: `python3 main.py`
- フェーズ限定試験: `python3 runs/orch/p1_p3.py`
- 保存先を変える試験: `python3 runs/orch/p3_p4.py --log-dir /tmp/cansat-p3-p4`
- GPS診断: `python3 runs/diag/gps.py`
- モータ診断: `python3 runs/diag/motor.py`

`runs/orch` の既定ログ先はデバッグラベルで分類する。ラベルは `--debug-label` で変更できる。

## 本番実行と systemd

`main.py` は標準入力や設定上書き引数を前提にしないため、systemd自動起動に向いた入口になっている。

- 本番入口は `python3 main.py` だけで完結する。
- 目標座標と無線設定はGit管理外の `mission.toml` だけから読む。
- `mission.toml` がない、または不正な場合はハードウェア初期化前に終了する。
- 本番入口では引数やプロセス環境変数による設定上書きを行わない。
- ログ出力先は絶対パスなので、systemdの `WorkingDirectory` に依存しない。

```ini
[Service]
WorkingDirectory=/home/pi/NSE2026
ExecStart=/home/pi/NSE2026/venv/bin/python3 /home/pi/NSE2026/main.py
Restart=on-failure
```

## 設定値の管理場所

- 実機で常に同じ値
  - `mission/const.py`
  - GPIO、モータ方向、モータ補正、カメラ向き、GPS補正など
- ミッションごとに変わる値
  - Git管理外の `mission.toml`
  - 目標座標と無線設定
- フェーズ試験だけで変える値
  - `runs/orch` の引数
  - デバッグラベルとログ保存先

設定変更後は、関連する診断スクリプトと自動テストを実行する。モータ方向や補正を変えた場合は、車輪を浮かせた状態または低速から前進・後退・左右旋回を確認する。

## 保守ルール

- ハードウェア固定値は `mission/const.py` に集約し、各フェーズへ散らさない。
- 試験日だけ変わる値は `mission.toml` または診断用引数へ置く。
- フェーズ仕様の変更は `mission/phases/` を優先して直す。
- デバッグ手順の追加は `runs/` に置き、本番入口には混ぜない。
- ログ列を変えた場合は `runs/spec/log_schema.py` と解析コードを同時に確認する。
