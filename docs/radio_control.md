# Mission Radio Control

この機能は、ドローン格納中の無線干渉を避けるために、`mission.env` で指定されたときだけ Raspberry Pi Zero 2 W の Wi-Fi を停止し、放出判定またはタイムアウトで復帰させるためのものです。

## 設計方針

- `main.py` は手動実行でも systemd 実行でもリポジトリ直下の `mission.env` を自動で読む。
- `mission.env` の `CANSAT_RADIO_CONTROL=mission` のときだけ有効にする。
- `CANSAT_RADIO_CONTROL=off` なら何もしない。
- センサー初期化とログファイル作成が終わってから、LEDで数秒合図して Wi-Fi を切る。
- `Phase0 -> Phase1` 遷移時に Wi-Fi を復帰する。
- `TIMEOUT_PHASE_0` 相当の絶対期限でも Wi-Fi を復帰する。
- Wi-Fi 操作に失敗してもミッション制御は止めない。

## mission.env 設定

リポジトリ直下の `mission.env.example` を各機体で `mission.env` にコピーする。

```bash
cd ~/NSE2026
cp mission.env.example mission.env
nano mission.env
```

通常試験では以下のままにする。

```bash
CANSAT_RADIO_CONTROL=off
```

本番または本番相当試験だけ以下にする。

```bash
CANSAT_RADIO_CONTROL=mission
```

`main.py` が自動で `./mission.env` を読むので、手動実行でも systemd 実行でもこのファイルだけを切り替えればよい。

## systemd 設定

`cansat.service` は `mission.env` を直接指定しなくてよい。

```ini
[Service]
WorkingDirectory=/home/pi/NSE2026
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/pi/NSE2026/venv/bin/python /home/pi/NSE2026/main.py
```

別の場所にある設定ファイルを読ませたい場合だけ、環境変数でパスを指定する。

```ini
Environment=CANSAT_MISSION_ENV_PATH=/home/pi/NSE2026/mission.env
```

Wi-Fi 操作は `rfkill block wifi` / `rfkill unblock wifi` を使う。権限が足りない場合は、サービスを root で動かすか、`rfkill` だけ passwordless sudo を許可して `mission.env` に次を設定する。

```bash
CANSAT_RADIO_USE_SUDO=1
```

## 動作の流れ

1. 手動または systemd が `main.py` を起動する。
2. センサー、カメラ、GPIO、ログファイル、バックグラウンドスレッドを初期化する。
3. LED を赤緑交互に点滅し、Wi-Fi 停止直前であることを示す。
4. `rfkill block wifi` で Wi-Fi を停止する。
5. Phase0 が放出/落下/着地系の判定を行う。
6. 判定成立、Phase0タイムアウト、または絶対復帰期限で `rfkill unblock wifi` を実行する。
7. 通常ミッションを継続し、復帰後に SSH でログを回収できる。

## 単体試験

実機の Wi-Fi を切らずにロジックを確認する。

```bash
python3 runs/spec/radio_control.py
python3 runs/spec/p0_detect.py
```

確認できること:

- `mission.env` または環境変数が `mission` でない限り Wi-Fi 操作を呼ばない。
- mission モードでは Phase0 開始前に block を呼ぶ。
- Phase0 復帰または絶対期限で unblock を呼ぶ。
- Phase0 の通常判定とタイムアウトが復帰処理を呼ぶ。

## 手動試験

systemdを使わず、手動ミッション実行でも `mission.env` だけで無線制御を有効化できる。

```bash
# mission.env
CANSAT_RADIO_CONTROL=mission
CANSAT_RADIO_DRY_RUN=0
CANSAT_RADIO_RESTORE_TIMEOUT_SEC=30
```

```bash
python3 main.py
```

コマンドだけ確認したい場合は `mission.env` で dry-run を使う。

```bash
CANSAT_RADIO_DRY_RUN=1
```

ミッション本体を走らせずに Wi-Fi 停止/復帰だけを試す場合は診断入口を使う。これは `mission.env` ではなく診断コマンドのオプションだけで動く。

```bash
python3 runs/diag/radio.py --duration 20
```

現在の `rfkill` 状態だけ見る。

```bash
python3 runs/diag/radio.py --status-only
```

## 実機試験

最初は dry run で systemd 配線だけ確認する。

```bash
# mission.env
CANSAT_RADIO_CONTROL=mission
CANSAT_RADIO_DRY_RUN=1
CANSAT_RADIO_RESTORE_TIMEOUT_SEC=30
```

```bash
sudo systemctl daemon-reload
sudo systemctl restart cansat.service
sudo journalctl -u cansat.service -f
```

ログに `Radio: DRY RUN rfkill block wifi` と `Radio: DRY RUN rfkill unblock wifi` が出れば、設定の読み込みは成功。

次に実際の Wi-Fi 復帰試験を行う。

```bash
# mission.env
CANSAT_RADIO_CONTROL=mission
CANSAT_RADIO_DRY_RUN=0
CANSAT_RADIO_RESTORE_TIMEOUT_SEC=30
```

PC側で ping を流しながら service を起動する。

```bash
ping <raspberry-pi-ip>
```

期待結果:

- 起動後、LED合図の後に ping が途切れる。
- 30秒程度で ping が戻る。
- SSH 再接続できる。
- `journalctl` に disable/restore の理由が残る。
- `log/<machine>/<run_id>/` に CSV が残り、`RadioDisabled` / `RadioControlMode` / `RadioLastEvent` / `RadioConfigSource` / `RadioRestoreDeadlineElapsedSec` が記録される。

CSV列:

- `RadioDisabled`: Wi-Fi停止中は `1`、通常時は `0`。
- `RadioControlMode`: 読み込まれた `CANSAT_RADIO_CONTROL`。本番停止時は `mission` になる。
- `RadioLastEvent`: 最後の無線制御イベント。例: `disabled:phase0_start`, `restored:phase0_release_altitude`, `restored:failsafe_timeout`。
- `RadioConfigSource`: 無線設定の読み込み元。`mission_env:/home/pi/NSE2026/mission.env` なら設定を読めている。
- `RadioRestoreDeadlineElapsedSec`: ミッション開始から見た絶対復帰期限。期限未設定時は `0.00`。

## デバッグ

状態確認:

```bash
rfkill list
sudo journalctl -u cansat.service -e
sudo systemctl status cansat.service
```

戻らない場合:

- `mission.env` の `CANSAT_RADIO_RESTORE_TIMEOUT_SEC` が長すぎないか確認する。
- CSVの `RadioControlMode` が `mission` か確認する。`off` や空なら、Wi-Fi停止は意図通り無効化されている。
- CSVの `RadioConfigSource` が `process_env` または `mission_env:...` か確認する。
- `CANSAT_RADIO_DRY_RUN=1` のまま本番試験していないか確認する。
- `rfkill unblock wifi` を手動実行する。
- 権限エラーが出る場合は root 実行か `CANSAT_RADIO_USE_SUDO=1` と sudoers 設定を確認する。

本番前チェック:

- `mission.env` が `CANSAT_RADIO_CONTROL=off` のとき、手動実行でも systemd でも Wi-Fi が切れない。
- `mission.env` が `CANSAT_RADIO_CONTROL=mission` かつ短い復帰期限のとき、手動実行でも systemd でも Wi-Fi が切れて戻る。
- 復帰後に SSH でログを読める。
