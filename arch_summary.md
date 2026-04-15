# NSE2026 Architecture Summary

このファイルは、`README.md` とは分けて管理する「リポジトリ構成・設計意図・二機体運用方針」の資料です。

- `README.md`
  - 原則としてセットアップ手順、依存導入、実行コマンドの入口だけを残す。
- `arch_summary.md`
  - ファイル構成、各ディレクトリの責務、二機体運用、共通化と固有化の境界、デバッグ運用をまとめる。

## ログ出力先早見表

| 実行入口 | 主な出力 | 既定の保存先 |
| --- | --- | --- |
| `python3 main.py --machine common` | ミッションCSV + 到達時PNG | `log/common/<run_id>/` |
| `python3 main.py --machine unit1` | ミッションCSV + 到達時PNG | `log/unit1/<run_id>/` |
| `python3 main.py --machine unit2` | ミッションCSV + 到達時PNG | `log/unit2/<run_id>/` |
| `runs/orch/*.py` | ミッションCSV + 到達時PNG | `runs/log/by_machine/<machine>/<label>/<run_id>/` |
| `runs/orch/*.py --debug-scope shared` | ミッションCSV + 到達時PNG | `runs/log/shared/<label>/<machine>/<run_id>/` |
| `runs/evt/open_parachute.py` | ミッションCSV + 到達時PNG | `runs/log/open_parachute/<run_id>/` |
| `runs/evt/landing_impact.py` | ミッションCSV + 到達時PNG | `runs/log/landing_impact/<run_id>/` |
| `runs/cam/cam_detector_dbg.py` | `debug.csv` + 各種PNG + 到達時PNG | `runs/log/camera_debug_<timestamp>_<ms>/` |
| `runs/cam/cam_capture_data.py` | 撮影JPEG群 | `runs/log/capture_<timestamp>[_session]/` |
| `runs/diag/gps.py` | 端末出力のみ | 保存ファイルなし |
| `runs/diag/sensor.py` | 端末出力のみ | 保存ファイルなし |
| `runs/diag/motor.py` | 端末出力のみ | 保存ファイルなし |
| `runs/diag/led.py` | 端末出力のみ | 保存ファイルなし |
| `runs/cam/cam_relay_sbc.py` | 端末出力 + 通信 | 保存ファイルなし |
| `runs/cam/cam_relay_pc.py` | 端末出力 + GUI表示 | 保存ファイルなし |
| `runs/spec/p0_detection.py` | `unittest` 出力 | 保存ファイルなし |

- `<run_id>` は `robust_log_YYYY-mmdd-HHMMSS-uuuuuu` 形式の実行ごとサブフォルダ。
- `main.py` 系と `runs/orch` 系では、CSV と `capture_reached.png` が同じ `<run_id>/` 配下に入る。
- `main.py` / `runs/orch` は `--log-dir` 指定時、そのパスを「実行一覧の親フォルダ」として使う。

## ファイル構成

```plaintext
├── README.md                           # セットアップ手順と基本実行コマンド
├── arch_summary.md                    # この資料
├── main.py                            # 本番ミッションの入口
├── .gitignore                         # Git除外設定
├── gerber/                            # 基板設計データ
│   └── NSE2026 v2_2026-04-13.zip
├── anlz/                              # ログ解析
│   ├── log_anlz.py
│   ├── explorer_map.py
│   └── DEBUG_POLICY.md
├── csmn/                              # ミッション本体
│   ├── __init__.py
│   ├── arch_summary.md
│   ├── const.py
│   ├── ctrl.py
│   ├── gps_util.py
│   ├── nav.py
│   ├── profile.py
│   ├── run.py
│   ├── st.py
│   ├── DEBUG_POLICY.md
│   ├── mgr/
│   │   ├── __init__.py
│   │   ├── hw_mgr.py
│   │   ├── led_mgr.py
│   │   ├── mtr_mgr.py
│   │   ├── sns_mgr.py
│   │   └── DEBUG_POLICY.md
│   └── phs/
│       ├── __init__.py
│       ├── base.py
│       ├── p0.py
│       ├── p1.py
│       ├── p2.py
│       ├── p3.py
│       ├── p4.py
│       ├── p5.py
│       ├── p6.py
│       ├── p7.py
│       └── DEBUG_POLICY.md
├── lib/                               # センサー・画像処理ライブラリ
│   ├── __init__.py
│   ├── bmp180.py
│   ├── bno055.py
│   ├── capture_roi_img.py
│   ├── detect_corn.py
│   └── DEBUG_POLICY.md
├── runs/                              # デバッグ・試験・補助実行スクリプト
│   ├── DEBUG_POLICY.md
│   ├── cam/
│   │   ├── cam_capture_data.py
│   │   ├── cam_detector_dbg.py
│   │   ├── cam_relay_pc.py
│   │   ├── cam_relay_readme.md
│   │   ├── cam_relay_sbc.py
│   │   └── DEBUG_POLICY.md
│   ├── diag/
│   │   ├── gps.py
│   │   ├── led.py
│   │   ├── motor.py
│   │   ├── sensor.py
│   │   └── DEBUG_POLICY.md
│   ├── evt/
│   │   ├── landing_impact.py
│   │   ├── open_parachute.py
│   │   └── DEBUG_POLICY.md
│   ├── orch/
│   │   ├── common.py
│   │   ├── orch_p0_log.py
│   │   ├── orch_p1_p3.py
│   │   ├── orch_p1_p7.py
│   │   ├── orch_p2_p3.py
│   │   ├── orch_p2_p7.py
│   │   ├── orch_p3_p4.py
│   │   ├── orch_p4_p7.py
│   │   └── DEBUG_POLICY.md
│   └── spec/
│       ├── p0_detection.py
│       └── DEBUG_POLICY.md
└── DEBUG_POLICY.md
```

## この構成の意図

- `main.py` は本番入口だけにする。
- 本番ロジックは `csmn/` に寄せる。
- デバッグや個別試験は `runs/` に逃がし、本番コードを書き換えて試験しない。
- センサーや画像処理の低レイヤは `lib/` に寄せる。
- ログ解析は `anlz/` に分け、実機制御と切り離す。

## 役割分担

- `README.md`
  - 環境構築、OS設定、依存導入、クローン、基本実行コマンド。
- `arch_summary.md`
  - 構成全体の把握、設計判断、保守時の境界確認。
- `csmn/arch_summary.md`
  - `csmn/` 内部の責務整理と変更ルール。
- `runs/cam/cam_relay_readme.md`
  - カメラ中継試験の詳細手順。

## 二機体運用の考え方

二機体を同じコードベースで扱うときは、次の3層で分ける。

- 共通ロジック
  - `csmn/` のフェーズ制御、状態管理、ナビゲーション、ハード制御。
- 機体固有プロファイル
  - `csmn/profile.py` に機体差分だけを置く。
  - 例: モータ補正、ログ出力先、カメラ向き、将来のGPIO差分。
- デバッグ入口
  - `main.py` と `runs/orch/*.py`、`runs/diag/*.py`。
  - どの機体をどの目的で動かすかを引数で切り替える。

初期プロファイル:

- `common`
  - 共通ベース設定。机上確認や未調整状態の入口。
- `unit1`
  - 1号機。
- `unit2`
  - 2号機。

## 共通化と固有化の境界

原則は「コードを複製せず、差分だけをプロファイルに閉じ込める」。

- 共通化するもの
  - フェーズ遷移
  - センサー取得ロジック
  - ログ形式
  - モータ制御アルゴリズム
  - デバッグスクリプトの入口構造
- 固有化するもの
  - モータ左右の補正値
  - カメラの制御向き補正
  - 目標座標の既定値
  - ログ保存先
  - 将来もし差が出るならGPIO割当や機体名

避けるべき運用:

- `unit1/` と `unit2/` のようにフォルダ丸ごと複製すること
- `main.py` をテストのたびに書き換えること
- 機体差分を `const.py` や各フェーズ実装に直接散らすこと

## デバッグ運用

デバッグは「共有デバッグ」と「機体固有デバッグ」を分けて考える。

- 共有デバッグ
  - 共通コードの不具合を潰すための試験。
  - 同じ条件で両機体を比較しやすい形でログを残す。
  - ログ保存先: `runs/log/shared/<label>/<machine>/`
- 機体固有デバッグ
  - 片方の個体差、組み付け差、キャリブレーション差を潰すための試験。
  - ログ保存先: `runs/log/by_machine/<machine>/<label>/`

この切り方により、人の運用負荷を下げつつ、あとで「共通問題か個体問題か」を見返しやすくする。

## 実行入口

- 本番実行
  - `python3 main.py --machine unit1`
  - `python3 main.py --machine unit2`
- フェーズ限定試験
  - `python3 runs/orch/orch_p1_p3.py --machine unit1`
  - `python3 runs/orch/orch_p3_p4.py --machine unit2 --debug-scope shared`
- 個別診断
  - `python3 runs/diag/gps.py --machine unit1`
  - `python3 runs/diag/motor.py --machine unit2`

## 本番実行と systemd

`main.py` は標準入力を前提にせず、引数だけで起動できるため、systemd 自動起動に向いた入口になっている。

- 本番入口は `python3 main.py` だけで完結する。
- 機体切替は `--machine` もしくは `CANSAT_MACHINE` で指定できる。
- 目標座標は `--target-lat` / `--target-lng`、または `CANSAT_TARGET_LAT` / `CANSAT_TARGET_LNG` で上書きできる。
- ログ出力先は `--log-dir`、または `CANSAT_LOG_DIR` で上書きできる。
- `csmn/profile.py` の既定 `LOG_DIR` はリポジトリ基準の絶対パスになるため、systemd の `WorkingDirectory` に過度に依存しない。

systemd 運用で意識する点:

- 実機の Python 実行環境を `ExecStart` で明示する。
  - 例: `ExecStart=/home/pi/NSE2026/venv/bin/python3 /home/pi/NSE2026/main.py --machine unit1`
- 依存ライブラリを入れた仮想環境またはシステム Python を使う。
  - `cv2`、`gpiozero`、`pynmea2` などが必要。
- 可能なら `WorkingDirectory` はリポジトリルートに合わせる。
  - 現状はログ出力先は絶対化されるが、保守しやすさのために合わせておくほうがよい。
- 再起動方針は systemd 側で制御する。
  - 例: `Restart=on-failure`

運用例:

```ini
[Service]
WorkingDirectory=/home/pi/NSE2026
ExecStart=/home/pi/NSE2026/venv/bin/python3 /home/pi/NSE2026/main.py
Environment=CANSAT_MACHINE=unit1
Environment=CANSAT_TARGET_LAT=30.374217
Environment=CANSAT_TARGET_LNG=130.959968
Environment=CANSAT_LOG_DIR=/home/pi/NSE2026/log/unit1
Restart=on-failure
```

この形にしておくと、systemd の unit file または `EnvironmentFile` で機体ごとの差分を切り替えられる。

## 固有値の管理場所と変更方法

固有値は「どこに置くか」を先に決めてから変更する。

- 機体ごとに違う既定値
  - `csmn/profile.py`
  - `PROFILE_REGISTRY["unit1"]`, `PROFILE_REGISTRY["unit2"]` の `const_overrides` に置く。
- 全機体で共通の既定値
  - `csmn/const.py`
  - まだ機体差分が不要な値はここに置く。
- 起動時だけ一時的に上書きしたい値
  - `main.py` の引数、または環境変数で上書きする。

優先順位は次の通り。

1. `main.py` の起動引数
2. 環境変数
3. `csmn/profile.py` の機体既定値
4. `csmn/const.py` の共通既定値

代表例:

- 目標座標
  - 共通の既定値: `csmn/const.py` の `TARGET_LAT`, `TARGET_LNG`
  - 機体ごとの既定値: `csmn/profile.py` の `const_overrides` に `TARGET_LAT`, `TARGET_LNG` を追加
  - 一時変更: `--target-lat`, `--target-lng` または `CANSAT_TARGET_LAT`, `CANSAT_TARGET_LNG`
- ログ保存先
  - 機体ごとの既定値: `csmn/profile.py` の `LOG_DIR`
  - 一時変更: `--log-dir` または `CANSAT_LOG_DIR`
- モータ補正値
  - 機体ごとの既定値: `csmn/profile.py` の `MOTOR_SPEED_SCALE_1`, `MOTOR_SPEED_SCALE_2`, `MOTOR_SPEED_OFFSET_1`, `MOTOR_SPEED_OFFSET_2`
  - 一時変更: 原則 `csmn/profile.py` を編集して管理する
- カメラ向き補正
  - 機体ごとの既定値: `csmn/profile.py` の `CAMERA_CONTROL_INVERT_X`
- 将来のGPIO差分
  - 差分が本当に固定化したら `csmn/profile.py` に持たせる
  - 共通で済むなら `csmn/const.py` のままにする

変更の実務ルール:

- 「その機体では常にそう動くべき値」は `csmn/profile.py` に入れる。
- 「試験日だけ変えたい値」は起動引数か環境変数で変える。
- 「二機体とも同じ基準で見直したい値」は `csmn/const.py` に置く。
- 値変更後は、共有デバッグか機体固有デバッグかを決めてログ保存先も対応させる。

## 保守ルール

- 値の追加候補が機体差分なら、まず `csmn/profile.py` に置けないか考える。
- 値の追加候補が全機体共通なら、`csmn/const.py` に置く。
- フェーズ仕様の変更は `csmn/phs/` を優先して直す。
- デバッグ手順の追加は `runs/` に置き、本番入口には混ぜない。
