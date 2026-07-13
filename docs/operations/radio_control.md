# Mission Radio Control

ドローン格納中の無線干渉を避けるため、`mission.toml`で有効にした場合だけ Raspberry Pi Zero 2 W のWi-FiをPhase0中に停止し、放出判定またはタイムアウトで復帰させる。

## mission.toml設定

リポジトリ直下のサンプルを各実機でコピーする。`mission.toml`はGit管理外であり、ゴール座標と無線設定の唯一の読込元になる。

```bash
cd ~/NSE2026
cp mission.toml.example mission.toml
nano mission.toml
```

通常試験ではWi-Fi制御を無効にする。

```toml
[radio]
control = "off"
pre_off_delay_sec = 3
restore_timeout_sec = 180
use_sudo = true
dry_run = false
```

本番または本番相当試験では `control = "mission"` にする。短時間の復帰試験だけ `restore_timeout_sec` を30などへ短縮する。コマンドだけを確認するときは `dry_run = true` にする。

設定値の意味:

- `control`: `"off"` または `"mission"`
- `pre_off_delay_sec`: Wi-Fi停止前のLED予告時間
- `restore_timeout_sec`: Wi-Fi停止から強制復帰までの秒数
- `use_sudo`: `rfkill`を`sudo -n`経由で実行するか
- `dry_run`: `rfkill`を実行せずコマンドだけ表示するか

値が欠けている、型が違う、未知のキーがある場合、ミッションはハードウェア初期化前に終了する。環境変数や起動引数による上書きは行わない。

## systemd設定

設定ファイルのパスはリポジトリ直下に固定されているため、systemdへ環境変数を追加する必要はない。

```ini
[Service]
WorkingDirectory=/home/pi/NSE2026
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/pi/NSE2026/venv/bin/python /home/pi/NSE2026/main.py
```

## rfkillのpasswordless sudo設定

通常ユーザーでミッションを動かし、Wi-Fi操作だけにsudoを許可する場合、まず実際のパスを確認する。

```bash
command -v rfkill
```

次に専用のsudoersファイルを作る。

```bash
sudo visudo -f /etc/sudoers.d/cansat-rfkill
```

`command -v rfkill`が`/usr/sbin/rfkill`を返し、実行ユーザーが`pi`の場合の例:

```sudoers
pi ALL=(root) NOPASSWD: /usr/sbin/rfkill block wifi, /usr/sbin/rfkill unblock wifi
```

構文を確認する。

```bash
sudo visudo -cf /etc/sudoers.d/cansat-rfkill
sudo -n rfkill block wifi
sudo -n rfkill unblock wifi
```

SSH接続中にblockを実行すると切断されるため、復帰手段を用意して実施する。

## 動作の流れ

1. `main.py`が`mission.toml`全体を読み、座標と無線設定を検証する。
2. センサー、カメラ、GPIO、ログ、バックグラウンドスレッドを初期化する。
3. Phase0開始時、LEDで停止予告を表示する。
4. `rfkill block wifi`でWi-Fiを停止する。
5. Phase1への遷移または復帰期限で`rfkill unblock wifi`を実行する。
6. 復帰後も通常ミッションを継続する。

Wi-Fi操作に失敗した場合はイベントをログへ残し、ミッション制御自体は継続する。

## 試験

ロジック試験:

```bash
python3 runs/spec/mission_config.py
python3 runs/spec/radio_control.py
python3 runs/spec/p0_detect.py
```

ミッション本体を動かさず、診断コマンドだけで停止・復帰を試す:

```bash
python3 runs/diag/radio.py --duration 20 --dry-run
python3 runs/diag/radio.py --duration 10 --use-sudo
```

診断コマンド固有の引数は`mission.toml`を書き換えず無線機能単体を確認するためのもので、本番設定の上書きには使われない。

## ログとトラブルシュート

CSVでは次を確認する。

- `RadioDisabled`: Wi-Fi停止中は`1`
- `RadioControlMode`: `off`または`mission`
- `RadioLastEvent`: 最後の停止・復帰イベント
- `RadioConfigSource`: 読み込んだ`mission.toml`の絶対パス
- `RadioRestoreDeadlineElapsedSec`: 復帰期限

状態確認:

```bash
rfkill list
sudo journalctl -u cansat.service -e
sudo systemctl status cansat.service
```

Wi-Fiが戻らない場合は、`restore_timeout_sec`、`dry_run`、sudoers設定、CSVの`RadioLastEvent`を確認し、必要なら`rfkill unblock wifi`をローカル操作で実行する。
