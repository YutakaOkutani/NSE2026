# NSE2026 アーキテクチャ概要

この文書は、`README.md` とは分けて管理する「リポジトリ構成・設計意図・二機体運用方針」の資料です。

- `README.md`
  - 原則としてセットアップ手順、依存導入、実行コマンドの入口だけを残す。
- `docs/architecture/overview.md`
  - ファイル構成、各ディレクトリの責務、二機体運用、共通化と固有化の境界、デバッグ運用をまとめる。

## ログ出力先早見表

| 実行入口 | 主な出力 | 既定の保存先 |
| --- | --- | --- |
| `python3 main.py --machine common` | ミッションCSV + 到達時PNG | `log/common/<run_id>/` |
| `python3 main.py --machine unit1` | ミッションCSV + 到達時PNG | `log/unit1/<run_id>/` |
| `python3 main.py --machine unit2` | ミッションCSV + 到達時PNG | `log/unit2/<run_id>/` |
| `runs/orch/*.py` | ミッションCSV + 到達時PNG | `runs/log/by_machine/<machine>/<label>/<run_id>/` |
| `runs/orch/*.py --debug-scope shared` | ミッションCSV + 到達時PNG | `runs/log/shared/<label>/<machine>/<run_id>/` |
| `runs/cam/detect_dbg.py` | `debug.csv` + 各種PNG + 到達時PNG | `runs/log/camera_debug_<timestamp>_<ms>/` |
| `runs/cam/capture.py` | 撮影JPEG群 | `runs/log/capture_<timestamp>[_session]/` |
| `runs/diag/gps.py` | 端末出力のみ | 保存ファイルなし |
| `runs/diag/sensor.py` | 端末出力のみ | 保存ファイルなし |
| `runs/diag/motor.py` | 端末出力のみ | 保存ファイルなし |
| `runs/diag/led.py` | 端末出力のみ | 保存ファイルなし |
| `runs/cam/relay_sbc.py` | 端末出力 + 通信 | 保存ファイルなし |
| `runs/cam/relay_pc.py` | 端末出力 + GUI表示 | 保存ファイルなし |
| `runs/spec/p0_detect.py` | `unittest` 出力 | 保存ファイルなし |

- `<run_id>` は `robust_log_YYYY-mmdd-HHMMSS-uuuuuu` 形式の実行ごとサブフォルダ。
- `main.py` 系と `runs/orch` 系では、CSV と `capture_reached.png` が同じ `<run_id>/` 配下に入る。
- `main.py` / `runs/orch` は `--log-dir` 指定時、そのパスを「実行一覧の親フォルダ」として使う。

## ファイル構成

```plaintext
├── README.md                           # セットアップ手順と基本実行コマンド
├── main.py                            # 本番ミッションの入口
├── .gitignore                         # Git除外設定
├── docs/                              # 設計・運用ドキュメント
│   ├── index.md                       # ドキュメント入口
│   ├── architecture/
│   │   └── overview.md                # この資料
│   ├── development/
│   │   ├── principles.md
│   │   ├── mission.md
│   │   ├── testing.md
│   │   └── components.md
│   ├── operations/
│   │   ├── camera_relay.md
│   │   └── radio_control.md
│   └── telemetry/
│       ├── specification.md
│       └── debugging.md
├── gerber/                            # 基板設計データ
│   └── NSE2026 v2_2026-04-13.zip
├── anlz/                              # ログ解析
│   ├── log.py
│   └── explorer.py
├── csmn/                              # ミッション本体
│   ├── __init__.py
│   ├── const.py
│   ├── ctrl.py
│   ├── gps_util.py
│   ├── nav.py
│   ├── profile.py
│   ├── run.py
│   ├── st.py
│   ├── mgr/
│   │   ├── __init__.py
│   │   ├── hw_mgr.py
│   │   ├── led_mgr.py
│   │   ├── mtr_mgr.py
│   │   ├── radio_mgr.py
│   │   └── sns_mgr.py
│   └── phs/
│       ├── __init__.py
│       ├── base.py
│       └── p0.py ... p7.py
├── lib/                               # センサー・画像処理ライブラリ
│   ├── __init__.py
│   ├── bmp180.py
│   ├── bno055.py
│   ├── roi_capture.py
│   └── cone_detect.py
├── runs/                              # デバッグ・試験・補助実行スクリプト
│   ├── cam/
│   │   ├── capture.py
│   │   ├── detect_dbg.py
│   │   ├── relay_pc.py
│   │   └── relay_sbc.py
│   ├── diag/
│   │   ├── gps.py
│   │   ├── led.py
│   │   ├── motor.py
│   │   ├── radio.py
│   │   └── sensor.py
│   ├── orch/
│   │   ├── common.py
│   │   ├── p0_log.py
│   │   └── p*_p*.py
│   ├── spec/
│   │   ├── log_schema.py
│   │   ├── p0_detect.py
│   │   └── radio_control.py
│   └── telemetry/
│       ├── README.md
│       ├── telemetry_pc.py
│       └── telemetry_sbc.py
```

## この構成の意図

- `main.py` は本番入口だけにする。
- 本番ロジックは `csmn/` に寄せる。
- デバッグや個別試験は `runs/` に逃がし、本番コードを書き換えて試験しない。
- センサーや画像処理の低レイヤは `lib/` に寄せる。
- ログ解析は `anlz/` に分け、実機制御と切り離す。
- 設計判断や運用手順は `docs/` に集約し、コードの各階層へ散らさない。

## `csmn/` 内部の責務

- `const.py`: 全機体共通の定数と閾値
- `profile.py`: `common` / `unit1` / `unit2` の機体差分
- `st.py`: スレッドセーフな共有状態
- `nav.py`: 距離・方位などの副作用が少ない計算
- `gps_util.py`: GPS 入出力と NMEA 処理
- `ctrl.py`: 初期化、フェーズディスパッチ、実行範囲制限
- `run.py`: `run_full_mission()`、`run_phase_sequence()`、`run_single_phase()` の入口
- `mgr/`: ハード初期化、センサー取得、モータ、LED、無線制御
- `phs/`: フェーズ共通インターフェースと `p0` から `p7` の状態遷移

通常遷移は `0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7` とし、一部フェーズは `4 -> 3` のようなフォールバックを持つ。

## ファイル命名ポリシー

- ファイル名は小文字を基本とし、単語区切りが必要な場合だけ `_` を使う。
- 実行入口は打ち込みやすさを優先し、ディレクトリ名で意味が分かる接頭辞は省く。
- 省略しすぎて役割が分からなくなる名前は避ける。
- ログフォルダ名や出力ファイル名は解析互換のため原則変更しない。
- 制御本体の責務が変わらない限り、リネームだけを目的にした内部構造変更は行わない。

## ファイル名変更対応表

| 変更前 | 変更後 | 理由 |
| --- | --- | --- |
| `anlz/log_anlz.py` | `anlz/log.py` | `anlz/` 配下で解析用途が明確なため短縮 |
| `anlz/explorer_map.py` | `anlz/explorer.py` | 探査再構成解析として短縮 |
| `lib/capture_roi_img.py` | `lib/roi_capture.py` | ROI 撮影用途を保ったまま短縮 |
| `lib/detect_corn.py` | `lib/cone_detect.py` | 検出対象を cone として明確化 |
| `runs/cam/cam_capture_data.py` | `runs/cam/capture.py` | `runs/cam/` 配下なので `cam_` を省略 |
| `runs/cam/cam_detector_dbg.py` | `runs/cam/detect_dbg.py` | デバッグ検出入口として短縮 |
| `runs/cam/cam_relay_pc.py` | `runs/cam/relay_pc.py` | `cam_` を省略し役割は維持 |
| `runs/cam/cam_relay_sbc.py` | `runs/cam/relay_sbc.py` | `cam_` を省略し役割は維持 |
| `runs/cam/cam_relay_readme.md` | `docs/operations/camera_relay.md` | カメラ中継手順を docs に集約 |
| `runs/orch/orch_p0_log.py` | `runs/orch/p0_log.py` | `runs/orch/` 配下なので `orch_` を省略 |
| `runs/orch/orch_p1_p3.py` | `runs/orch/p1_p3.py` | 同上 |
| `runs/orch/orch_p1_p7.py` | `runs/orch/p1_p7.py` | 同上 |
| `runs/orch/orch_p2_p3.py` | `runs/orch/p2_p3.py` | 同上 |
| `runs/orch/orch_p2_p7.py` | `runs/orch/p2_p7.py` | 同上 |
| `runs/orch/orch_p3_p4.py` | `runs/orch/p3_p4.py` | 同上 |
| `runs/orch/orch_p4_p7.py` | `runs/orch/p4_p7.py` | 同上 |
| `runs/spec/p0_detection.py` | `runs/spec/p0_detect.py` | 意味を保ったまま短縮 |

## 機体判別仕様

機体判別は `csmn/profile.py` の `resolve_machine_profile()` に集約する。

判別順序:

1. `--machine` などコードから渡された明示指定
2. リポジトリ直下の `machine.txt`
3. 判別不能時の `common`

明示指定または `machine.txt` に未知の機体名が入っていた場合は `ValueError` とし、誤った補正値で走らせない。どちらもない場合だけ、機体固有補正を含まない `common` をフォールバックにする。

ログ分離は既存のまま維持する。`main.py` は `log/<machine>/`、`runs/orch/` は `runs/log/by_machine/<machine>/<label>/` または `runs/log/shared/<label>/<machine>/` を使う。

## コメント追加方針

- 機体判別、プロファイル適用、ログ出力先確定の意図が分かる箇所に短いコメントを追加する。
- モータ補正など安全に関わる診断入口では、本番と同じプロファイル適用経路を使う意図を明記する。
- 制御式、タイミング、通信形式、ログ列の意味を変えるコメント追加は行わない。
- コードをそのまま言い換えるコメントは避け、将来の保守者が判断理由を追える内容にする。

## 今回の変更点

- 長い実行ファイル名を短縮し、参照ドキュメントと import を追従した。
- `csmn/profile.py` に機体判別処理を追加し、判別結果を本番・オーケストレーション・GPS診断・モータ診断で共通利用するようにした。
- `--machine` は明示上書きとして維持し、指定がない場合は `machine.txt` を読む動作へ変更した。
- 判別不能時のフォールバックと、未知の明示指定をエラーにする条件を明文化した。
- 機体判別と安全上重要なプロファイル適用箇所にコメントを追加した。

## 役割分担

- `README.md`
  - 環境構築、OS設定、依存導入、クローン、基本実行コマンド。
- `docs/index.md`
  - 目的別に各文書へ移動するための目次。
- `docs/architecture/overview.md`
  - 構成全体の把握、設計判断、保守時の境界確認。
- `docs/development/`
  - 開発原則と領域別の設計・デバッグ指針。
- `docs/operations/camera_relay.md`
  - カメラ中継試験の詳細手順。
- `docs/operations/radio_control.md`
  - ミッション中の Wi-Fi 停止・復帰手順。
- `docs/telemetry/`
  - テレメトリ仕様とデバッグ手順。

## 二機体運用の考え方

二機体を同じコードベースで扱うときは、次の3層で分ける。

- 共通ロジック
  - `csmn/` のフェーズ制御、状態管理、ナビゲーション、ハード制御。
- 機体固有プロファイル
  - `csmn/profile.py` に機体差分だけを置く。
  - 例: モータ補正、ログ出力先、将来のGPIO差分。
- デバッグ入口
  - `main.py` と `runs/orch/*.py`、`runs/diag/*.py`。
  - どの機体をどの目的で動かすかを `machine.txt` または引数で切り替える。

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
  - ログ保存先
  - 将来もし差が出るならGPIO割当や機体名
- 共通値として扱うもの
  - カメラの制御向き補正
  - GPS方位補正、GPS座標補正
  - 目標座標の既定値

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
  - `python3 runs/orch/p1_p3.py --machine unit1`
  - `python3 runs/orch/p3_p4.py --machine unit2 --debug-scope shared`
- 個別診断
  - `python3 runs/diag/gps.py --machine unit1`
  - `python3 runs/diag/motor.py --machine unit2`

## 本番実行と systemd

`main.py` は標準入力を前提にせず、引数だけで起動できるため、systemd 自動起動に向いた入口になっている。

- 本番入口は `python3 main.py` だけで完結する。
- 機体切替は `--machine` を最優先し、指定がなければリポジトリ直下の `machine.txt` を読む。
- `machine.txt` に `unit1` または `unit2` と書くと、引数なしでも機体を固定できる。
- 目標座標は `--target-lat` / `--target-lng`、または `CANSAT_TARGET_LAT` / `CANSAT_TARGET_LNG` で上書きできる。
- ログ出力先は `--log-dir`、または `CANSAT_LOG_DIR` で上書きできる。
- `csmn/profile.py` の既定 `LOG_DIR` はリポジトリ基準の絶対パスになるため、systemd の `WorkingDirectory` に過度に依存しない。

systemd 運用で意識する点:

- 実機の Python 実行環境を `ExecStart` で明示する。
  - 例: `ExecStart=/home/pi/NSE2026/venv/bin/python3 /home/pi/NSE2026/main.py`
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
Environment=CANSAT_TARGET_LAT=30.374217
Environment=CANSAT_TARGET_LNG=130.959968
Environment=CANSAT_LOG_DIR=/home/pi/NSE2026/log/unit1
Restart=on-failure
```

機体名は `machine.txt` で固定し、systemd 側には試験日ごとに変わる値だけを置く。

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

機体名そのものの判別優先順位は「明示引数、`machine.txt`, `common`」である。リポジトリ直下の `machine.txt` はローカル設定として使い、Git には含めない。

代表例:

- 目標座標
  - 共通の既定値: `csmn/const.py` の `TARGET_LAT`, `TARGET_LNG`
  - 一時変更: `--target-lat`, `--target-lng` または `CANSAT_TARGET_LAT`, `CANSAT_TARGET_LNG`
- ログ保存先
  - 機体ごとの既定値: `csmn/profile.py` の `LOG_DIR`
  - 一時変更: `--log-dir` または `CANSAT_LOG_DIR`
- モータ補正値
  - 機体ごとの既定値: `csmn/profile.py` の `MOTOR_SPEED_SCALE_1`, `MOTOR_SPEED_SCALE_2`, `MOTOR_SPEED_OFFSET_1`, `MOTOR_SPEED_OFFSET_2`
  - 一時変更: 原則 `csmn/profile.py` を編集して管理する
- カメラ向き補正、GPS補正
  - 共通の既定値: `csmn/const.py` の `CAMERA_CONTROL_INVERT_X`, `GPS_HEADING_OFFSET`, `GPS_COORD_LAT_OFFSET_DEG`, `GPS_COORD_LNG_OFFSET_DEG`
- 将来のGPIO差分
  - 差分が本当に固定化したら `csmn/profile.py` に持たせる
  - 共通で済むなら `csmn/const.py` のままにする

変更の実務ルール:

- 「その機体では常にそう動くべき値」は `csmn/profile.py` に入れる。
- 「試験日だけ変えたい値」は起動引数か環境変数で変える。
- 「二機体とも同じ基準で見直したい値」は `csmn/const.py` に置く。カメラ向き、GPS補正、目標座標はこの扱いにする。
- 値変更後は、共有デバッグか機体固有デバッグかを決めてログ保存先も対応させる。

## 保守ルール

- 値の追加候補が機体差分なら、まず `csmn/profile.py` に置けないか考える。
- 値の追加候補が全機体共通なら、`csmn/const.py` に置く。
- フェーズ仕様の変更は `csmn/phs/` を優先して直す。
- デバッグ手順の追加は `runs/` に置き、本番入口には混ぜない。
