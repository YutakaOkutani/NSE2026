# NSE2026

## 1. 環境構築の準備

### ハードウェア

* Raspberry Pi Zero 2 W
* microSDカード（16GB以上あれば良い）
* 電源（5V / 2.5A）（PCからのUSB給電でもよいが、不安定になるときがある）
* PC
* モニター・キーボード・USBハブ（あると便利）
* Mini HDMI ケーブル（モニター接続用）

### ソフトウェア

* Raspberry Pi Imager
* Git
* ターミナル
* WSL2（Windowsユーザーは推奨）

---

## 2. Raspberry Pi Zero 2 W のセットアップ

### 2.1 OS イメージの準備

1. Raspberry Pi Imager をインストールする（ <https://www.raspberrypi.com/software/> ）
2. 起動後、

   * Device → *Raspberry Pi Zero 2 W*
   * OS → *Raspberry Pi OS(other)*から*Raspberry Pi OS Lite（64-bit）* （GUIは要らない）
   * Storage → MicroSDカード

3. 以下を設定・有効化する

   * Hostname
   * Localisation（国・キーボード設定）
   * User
   * Wi-Fi
   * Remote access（SSHの有効化、パスワード認証で十分）
   * Raspberry Pi Connect（オフでよい）

4. Writing（書き込み）を実行

---

### 2.2 初回起動と接続

1. microSD を ラズパイ に挿入して電源投入
2. PC から次のコマンドで接続

    ```bash
    ssh ユーザー名@ホスト名.local（or IPアドレス）
    ```

3. パスワードは Imager で設定したものを使用

### 2.3 接続できない場合は以下を確認

1. 同一ネットワークにいるか
2. `.local` 解決ができない環境では、ラズパイの IPアドレス を確認して、ホスト名のところを IP に置き換えて接続する（WindowsやAndroid端末では、mDNS（.local）が安定的にサポートされておらず、ホスト名接続は一般に不安定）
3. Permission denied (publickey,password).が出る場合、以下のコマンドで接続

    ```bash
    ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no ユーザー名@IPアドレス
    ```

4. その後、以下のコマンドでラズパイ上の設定を確認

    ```bash
    sudo nano /etc/ssh/sshd_config
    ```

5. その中に、以下の行があれば確認。

    ```plaintext
    PasswordAuthentication yes
    ```

6. PasswordAuthentication が no になっていたら yes に変更。

7. "#" でコメントアウトされている場合は、"#" を外して PasswordAuthentication yes にする

8. 設定を変更したら、SSH サーバーを再起動。

    ```bash
    sudo systemctl restart ssh
    ```

---

### 2.3 初期アップデート

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

---

## 3. Python 実行環境の構築

### 3.1 依存パッケージのインストール

```bash
# ===== 基本ツール =====
sudo apt install -y git tmux i2c-tools curl

# ===== 実行に必要なPythonライブラリ（apt）=====
# GPIO / シリアル / I2C
sudo apt install -y python3-gpiozero python3-rpi-lgpio liblgpio1 python3-serial python3-smbus

# pipでライブラリを追加インストールするためのツール
sudo apt install -y python3-pip python3-setuptools

# カメラ / 画像処理（Picamera2 + OpenCV + NumPy）
sudo apt install -y python3-picamera2 python3-libcamera libcamera-apps python3-opencv python3-numpy

# OpenCVでGUI表示（imshow等）する場合に必要。ヘッドレス運用なら不要なことが多い
sudo apt install -y libgl1

# ===== ここから下は自前ビルドをするなら必要=====

# C拡張やライブラリをソースからビルドする場合に必要
# sudo apt install -y build-essential python3-dev swig

# lgpio を C で開発/コンパイルする場合のヘッダ（Pythonで動かすだけなら通常不要）
# sudo apt install -y liblgpio-dev

```

### 3.2 Python 仮想環境の作成

```bash
# 仮想環境の作成（システムパッケージを引き継ぐ --system-site-packages が重要）
# これにより、aptで入れた OpenCV や Picamera2 などがそのまま仮想環境内でも使用可能になる
python3 -m venv --system-site-packages venv

# 仮想環境の有効化
source venv/bin/activate

# pip自体の更新
pip install --upgrade pip

# GPS解析用ライブラリ
pip install pynmea2

# その他、必要なライブラリがあればここで追加
# pip install smbus2
```

---

## 4. シリアル通信 / GPIO / I2C の有効化

GPS は GPIO 14/15（物理ピン 8/10）の UART を使用する。Raspberry Pi Zero 2 W では、標準状態の `/dev/serial0` がクロック変動の影響を受ける mini UART（`ttyS0`）を指す場合がある。本機では Bluetooth を使用しないため、Bluetooth を無効化し、安定性の高い PL011 UART（`ttyAMA*`）を GPS 専用として割り当てる。

### 4.1 raspi-config での設定

```bash
sudo raspi-config
```

### 4.2 I2C と UART の設定

* Interface Options → I2C
* Interface Options → Serial Port

Serial Port では、表示される2つの質問に次のように回答する。

```text
Would you like a login shell to be accessible over serial?
→ No

Would you like the serial port hardware to be enabled?
→ Yes
```

シリアルログインを有効にすると、Linux のコンソールと GPS が同じ UART を使用して競合するため、必ず `No` にする。

### 4.3 シリアルコンソールが無効になっていることを確認

```bash
cat /proc/cmdline
```

出力に次のような指定が残っていないことを確認する。

```text
console=serial0,115200
console=ttyS0,115200
console=ttyAMA0,115200
```

残っている場合は、Bookworm 系 Raspberry Pi OS では `/boot/firmware/cmdline.txt`、旧OSでは `/boot/cmdline.txt` から該当する `console=...` だけを削除する。

```bash
sudo nano /boot/firmware/cmdline.txt
```

`cmdline.txt` は改行せず、必ず1行のまま保存する。

### 4.4 Bluetooth を無効化して PL011 UART を GPS に割り当てる

Bookworm 系 Raspberry Pi OS では次を編集する。

```bash
sudo nano /boot/firmware/config.txt
```

旧OSの場合は `/boot/config.txt` を編集する。ファイル末尾に次を追加する。

```ini
enable_uart=1
dtoverlay=disable-bt
```

設定を書き込めたことを確認する。

```bash
grep -E '^(enable_uart|dtoverlay=.*(disable-bt|miniuart-bt))' /boot/firmware/config.txt
```

期待する出力は次の2行である。`enable_uart=1` しか表示されない場合、`dtoverlay=disable-bt` はまだ設定されていない。

```text
enable_uart=1
dtoverlay=disable-bt
```

次に Bluetooth の UART 初期化サービスを確認する。

```bash
systemctl is-enabled hciuart.service
```

`enabled` と表示された場合は無効化・停止・マスクする。

```bash
sudo systemctl disable --now hciuart.service
sudo systemctl mask hciuart.service
```

`disabled` または `masked` なら追加操作は不要。Raspberry Pi OS のバージョンによってはサービス自体が存在せず `not-found` と表示されるが、この場合も無効化対象がないため追加操作は不要である。

本機では Bluetooth を使用しないため、`dtoverlay=miniuart-bt` ではなく `dtoverlay=disable-bt` を使用する。これにより、PL011 UART が GPIO 14/15 側の primary UART になる。

### 4.5 再起動

設定完了後に再起動する。

```bash
sudo reboot
```

### 4.6 UART 割り当ての確認

再起動後に次を実行する。

```bash
grep -E '^(enable_uart|dtoverlay=.*(disable-bt|miniuart-bt))' /boot/firmware/config.txt
systemctl is-enabled hciuart.service
ls -l /dev/serial0
readlink -f /dev/serial0
```

期待する状態は次のとおり。

* `enable_uart=1` と `dtoverlay=disable-bt` が表示される
* `hciuart.service` は `disabled`、`masked`、または `not-found`
* `/dev/serial0` の実体が `ttyS0` ではなく、`/dev/ttyAMA0` などの `ttyAMA*`（PL011 UART）になっている

```text
/dev/ttyAMA0
```

`/dev/serial0` が存在しない、または `ttyS0` を指したままの場合は、その状態で本番運用へ進まず、`enable_uart=1`、`dtoverlay=disable-bt`、編集した設定ファイルのパスを再確認する。

---

## 5. リポジトリのクローン

```bash
git clone https://github.com/YutakaOkutani/NSE2026
cd NSE2026
```

---

## 5.1 ミッション設定ファイルの作成

各 Raspberry Pi で、リポジトリ直下に `mission.toml` を作成し、その日のゴール座標と無線設定を記入する。このファイルはGit管理外なので、座標変更のコミットは不要。

```bash
cd ~/NSE2026
cp mission.toml.example mission.toml
nano mission.toml
```

設定項目と記入例は [`mission.toml.example`](mission.toml.example) を参照する。

`main.py` は座標を引数、環境変数、`mission/const.py` から取得しない。`mission.toml` がない、必須項目がない、型や範囲が不正、未知の項目がある場合は、ハードウェアを初期化せず終了する。起動前に必ず採用座標を確認する。

`mission.toml.example`の座標は意図的に範囲外にしてあり、コピーしただけでは起動できない。実機ではPython 3.11以降を使用する。

---

## 5.2 ドキュメント

セットアップと基本コマンドはこの `README.md`、設計・開発方針・運用手順は [`docs/index.md`](docs/index.md) を入口として参照する。

コードフォルダには入口として必要な `README.md` だけを置き、それ以外の Markdown 文書は `docs/` 配下で管理する。

---

## 5.3 実行コマンド早見表

前提:

* 作業ディレクトリは `~/NSE2026`
* 必要なら先に仮想環境を有効化: `source venv/bin/activate`

```bash
# E2E試験時実行
python3 main.py

# パラ投下・着地衝撃試験時実行
python3 runs/orch/p0_p1.py

# 各種テストコード
python3 runs/diag/sensor.py
python3 runs/diag/sonar.py
python3 runs/diag/gps.py
python3 runs/diag/motor.py
python3 runs/diag/led.py

# 画像・カメラ系
# ROI参照画像（本番detectorが読む画像）を撮影
python3 lib/roi_capture.py --count 3 --interval 0.5

# 現場サンプル収集（学習・比較・記録用。本番ROIには自動反映されない）
python3 runs/cam/capture.py --count 10 --interval 0.5 --prefix sample

# 本番detectorのライブデバッグ
python3 runs/cam/detect_dbg.py --phase 4

# ログ解析（PC上で実行）
python3 analysis/log.py
```

超音波センサだけを確認する場合は `python3 runs/diag/sonar.py` を実行し、対象物を前後に動かして `VALID` の距離が追従することを確認する。`HOLD` は一時的な読取失敗、`STALE` は最終正常値から1秒以上更新されていない状態を示す。10秒で自動終了する場合は `--duration 10` を付ける。ECHO出力はGPIOへ直結せず、必ず3.3 Vへレベル変換する。

---

## 6. カメラの設定

### まずカメラ認識を確認

```bash
# 認識デバイスの一覧
rpicam-hello --list-cameras

# dmesg による初期化ログ確認
sudo dmesg | grep -Ei "ov5647|camera|unicam" | tail -n 30

# 初期化時間の確認
time rpicam-hello -t 1
```

カメラが認識されている場合、通常は `/boot/firmware/config.txt` を変更しない。

### OV5647 が自動認識されない場合のみ

Raspberry Pi Camera Module V1 系など、OV5647 センサーのカメラが自動認識されない場合だけ、手動指定を検討する。
OV5647 以外のカメラでは、この設定を入れない。

```bash
# 設定ファイルを編集
sudo nano /boot/firmware/config.txt

# OV5647を手動指定する場合のみ、以下を追加または修正
camera_auto_detect=0
dtoverlay=ov5647
```

編集後、再起動する。

```bash
sudo reboot
```

### カメラコマンドの確認

```bash
rpicam-hello --help
rpicam-still --help
rpicam-vid --help
```

### カメラ映像のテスト

```bash
# ライブプレビュー
rpicam-hello -t 0

# 静止画撮影
rpicam-still -o test.jpg

# 動画撮影
rpicam-vid -t 10000 -o test.h264

# 高解像度での静止画撮影テスト
rpicam-still --width 2592 --height 1944 -o maxres.jpg
```

## 7. 実行（テスト時）

### 本番用コード

```bash
# ===== 推奨: 最初からデタッチ状態で起動する方法 =====
# このコマンドで tmux セッション内に main.py を起動する。SSH が切れても tmux セッションが残るので実行継続できる
# ※ 重要: `python3 main.py` を tmux の外で起動すると、SSH切断時に一緒に止まる可能性がある
# ※ 重要: これは「SSH切断対策」。ラズパイの再起動/電源断まで含めて継続したい場合は、下の systemd 設定を使う
cd ~/NSE2026
tmux new-session -d -s cansat 'bash -lc "source venv/bin/activate && exec python3 main.py"'

# ログ/画面を確認したいときに接続（アタッチ）
tmux attach -t cansat

# 画面を閉じずに離脱（デタッチ）する操作
# キー操作: Ctrl+b を押してから d
# ※ `exit` / Ctrl+C は tmux セッション内のシェル/プログラムを終了させるので注意

# セッション一覧を確認（起動確認に使える）
tmux ls

# すでに cansat セッションが存在する場合は、新規作成せず接続して再利用
# tmux attach -t cansat
```

```bash
# ===== 対話的に起動する方法（手動操作したい場合） =====
cd ~/NSE2026
tmux new -s cansat
source venv/bin/activate
python3 main.py
# 離脱時は Ctrl+b → d
# 再接続後は `tmux attach -t cansat`
```

## 8. トラブルシューティング

### センサが認識されない

* `sudo i2cdetect -y 1` で確認
* 配線の導通を確認

---

## 9. 本番向けの設定

### 本番運用（systemd）

`tmux` は手動起動して画面を見ながらデバッグする場合、`systemd` はSSH切断後も継続する本番運用に使用する。定義本体は [`deploy/systemd/`](deploy/systemd/) で管理する。

各unitの役割は次のとおり。

* `cansat.service`: `main.py`を実行し、異常終了時に再起動する
* `cansat.timer`: OS起動から5分後に`cansat.service`を開始する
* `discord-ip.service`: DiscordへIPアドレスを通知する。ミッションのserviceとtimerには依存しない

#### 1. 実機のパスを確認する

```bash
cd ~/NSE2026
pwd
ls venv/bin/python
```

配布例はユーザー`pi`、配置先`/home/pi/NSE2026`を仮定している。異なる場合は、配置後の`User`、`Group`、`WorkingDirectory`、`ExecStart`を実環境に合わせて編集する。

#### 2. unitファイルを配置する

```bash
cd ~/NSE2026
sudo install -m 0644 deploy/systemd/cansat.service.example /etc/systemd/system/cansat.service
sudo install -m 0644 deploy/systemd/cansat.timer.example /etc/systemd/system/cansat.timer
sudo install -m 0644 deploy/systemd/discord-ip.service.example /etc/systemd/system/discord-ip.service

sudo nano /etc/systemd/system/cansat.service
sudo nano /etc/systemd/system/cansat.timer
sudo nano /etc/systemd/system/discord-ip.service
sudo systemctl daemon-reload
```

`cansat.timer`の`OnBootSec`は開始までの待ち時間で、配布例では`5min`としている。必要に応じて配置後の`/etc/systemd/system/cansat.timer`を編集する。`cansat.service`と`discord-ip.service`は、ユーザー名とパスを必ず確認する。

unit更新時は同じ`install`コマンドで再配置し、編集が必要な項目を再確認してから`sudo systemctl daemon-reload`を実行する。

#### 3. 自動起動を選択する

ミッションを起動から5分後に開始する場合は、`cansat.service`を直接enableせず、timerをenableする。

```bash
sudo systemctl enable cansat.timer
```

Discord通知をOS起動時に実行する場合は、Discord側だけをenableする。

```bash
sudo systemctl enable discord-ip.service
```

`enable`は次回起動時の設定だけを変更し、その場では実行しない。ミッションとDiscord通知は別々に有効・無効を管理できる。

#### 4. 手動で起動・停止する

```bash
# ミッションを今すぐ開始
sudo systemctl start cansat.service

# ミッションを停止
sudo systemctl stop cansat.service

# Discord通知を今すぐ実行
sudo systemctl start discord-ip.service

# ミッションの5分遅延自動起動だけを無効化
sudo systemctl disable --now cansat.timer
```

`cansat.timer`を無効化しても、`cansat.service`と`discord-ip.service`は手動で起動できる。Discordの自動起動設定にも影響しない。

#### 5. 状態とログを確認する

```bash
sudo systemctl status cansat.timer
sudo systemctl list-timers cansat.timer
sudo systemctl status cansat.service
sudo journalctl -u cansat.service -e
sudo journalctl -u cansat.service -f
sudo journalctl -u discord-ip.service -e
```

`discord-ip.service`は1回実行して終了する`Type=oneshot`なので、送信後に`inactive (dead)`と表示されるのは正常。送信結果は`journalctl`で確認する。

#### 6. 再起動後の動作を確認する

```bash
sudo reboot
```

再接続後に、今回の起動分だけを確認する。

```bash
sudo systemctl status cansat.timer
sudo systemctl list-timers cansat.timer
sudo systemctl status cansat.service
sudo journalctl -u cansat.service -b
sudo journalctl -u discord-ip.service -b
```

#### 7. よくある起動失敗ポイント

* `WorkingDirectory`または`ExecStart`が実際の配置と違う
* `/home/pi/NSE2026/venv/bin/python`が存在しない
* unitの`User`または`Group`が実機のユーザーと違う
* `mission.toml`または`discord.env`が作成されていない
* 仮想環境に必要なライブラリが入っていない
* `cansat.service`を直接enableして、意図せず起動直後にも実行している

`main.py`はリポジトリ直下の`mission.toml`を必須設定として読む。Wi-Fi停止・復帰の切り替えと試験方法は [`docs/operations/radio_control.md`](docs/operations/radio_control.md) を参照する。通常ユーザーのままWi-Fiを切る場合は、同文書に従い`rfkill`だけpasswordless sudoを許可する。

本番中に一時的に`tmux`でデバッグする場合は、先に`sudo systemctl stop cansat.service`を実行し、二重起動を避ける。

---

## 10. 便利なコマンドや設定

### 基本的なgit操作コマンド

```bash
# ファイルをステージングに追加
git add .
# コミットを作成
git commit -m "Initial commit"
# GitHub へ初回プッシュ
git push -u origin main
# 2回目以降
git push
```

#### ブランチの作成・切り替え

ブランチを切り替える前に、未コミットの変更がないか確認する。

```bash
git status
```

`main` から新しい作業ブランチを作成し、そのブランチへ切り替える場合:

```bash
git switch main
git pull --ff-only origin main
git switch -c refactor/remove-multi-airframe
```

すでにローカルに存在する作業ブランチへ切り替える場合:

```bash
git switch refactor/remove-multi-airframe
```

リモートにだけ存在する作業ブランチを初めてローカルへ取得する場合:

```bash
git fetch origin
git switch --track origin/refactor/remove-multi-airframe
```

現在のブランチと、ローカルに存在するブランチの一覧は次のコマンドで確認できる。先頭に `*` があるものが現在のブランチ。

```bash
git branch
git status --short --branch
```

作業ブランチを初めてGitHubへプッシュする場合:

```bash
git push -u origin refactor/remove-multi-airframe
```

Pull Requestのマージ後は `main` に戻り、最新版を取得してから不要になったローカルブランチを削除する。

```bash
git switch main
git pull --ff-only origin main
git branch -d refactor/remove-multi-airframe
```

未コミットの変更があると、安全のためブランチ切り替えが拒否される場合がある。その場合は変更をコミットするか、`git stash` で一時退避してから切り替える。作業中の変更を消す可能性があるため、安易に `--force` を付けない。

#### Raspberry Pi実機の推奨更新手順

実機はGitHub上の確定済みコードを実行する環境として扱い、原則として実機上ではソースコードの編集やコミットを行わない。更新には、履歴を自動で書き換えず、単純に最新版へ進める場合だけ成功する `pull --ff-only` を使う。

`main` へマージ済みの本番コードを更新する場合:

```bash
cd ~/NSE2026

# 更新中の自動起動と実行中コードを止め、現在のブランチと変更状態を確認
sudo systemctl stop cansat.timer cansat.service
git status --short --branch

# GitHubの情報を取得してmainへ切り替え、安全に最新版へ進める
git fetch origin
git switch main
git pull --ff-only origin main

# 更新内容と自動テストを確認
git log -1 --oneline
python3 runs/spec/test_all.py

# 更新後も停止状態を維持していることを確認
sudo systemctl status cansat.timer cansat.service
```

`git status` に `README.md` や `mission/*.py` などの変更が表示された場合は、そのまま更新しない。実機固有の `mission.toml` はGit管理外なので残るが、追跡対象ファイルの変更がある場合は、内容を確認してPC側へ退避または反映してから作業する。

更新手順では安全のためミッションを自動再開しない。機体を安全な状態に置き、本番運用を今すぐ開始する意図がある場合に限り、別途 `sudo systemctl start cansat.service` を実行する。`systemctl stop` はunitを無効化しないため、有効化済みのtimerは次回起動時にも設定どおり利用できる。

Pull Requestをマージする前に、実機で `refactor/remove-multi-airframe` を確認する場合:

```bash
cd ~/NSE2026
sudo systemctl stop cansat.timer cansat.service
git status --short --branch
git fetch origin

# ローカルにブランチがすでにある場合
git switch refactor/remove-multi-airframe
git pull --ff-only origin refactor/remove-multi-airframe

python3 runs/spec/test_all.py
sudo systemctl status cansat.timer cansat.service
```

リモートにだけ存在するブランチを実機で初めて取得する場合は、上記の `git switch` と `git pull` の代わりに次を実行する。

```bash
git switch --track origin/refactor/remove-multi-airframe
```

実機更新では通常 `git pull --rebase` や `git reset --hard` を使用しない。`rebase` はローカルコミットを積み直す開発作業向けであり、`reset --hard` は実機上の未退避変更を消すため、原因を確認したうえで復旧が必要な場合に限る。

---

### VPNサービスを使って、ラズパイのIPアドレスを固定化する方法（Tailscaleを使う方法）

#### 0. そもそも

前述のとおり、WindowsPCやAndroid端末は、mDNSが不安定なので、ラズパイとのSSH接続にはIPアドレスが必要

##### 仮想VPNサービス（ここではTailscale）を使えば

Tailscaleに登録された各デバイスは：

* 固定の仮想IPアドレスを持つ（100.x.y.z 形式）

* デバイスがオンラインの間、そのIPは常に同じ

* 管理画面に表示される

* そのIPで直接SSH接続が可能になる（実際のネットワークは同じでなくてもよい）

```powershell
ssh pi@100.x.y.z
```

#### 1. 構成手順

##### 0. 前提

PC: Windows（Macならそもそもこの問題は起きないので設定不要）
スマホ: Android

##### 1. アカウント作成（PCで）

[https://tailscale.com/](https://tailscale.com/)

* Google / GitHub / Microsoft などでログイン
* これが 仮想LAN になる

##### 2. Windows にインストール

[https://tailscale.com/download](https://tailscale.com/download)

* Windows版をDL
* インストール
* ログイン
* Tailscale はタスクトレイ常駐アプリとしてふるまう。

##### 3. スマホ にも入れる

* デスクトップで表示されるQRコードか Playストア で検索してインストール
* ログイン
* デスクトップに端末が追加されたか確認
* 案内されるテストコマンドをPCで実行して接続を確認できる

```powershell
ping 100.x.y.z
```

---

##### 4.  Raspberry Pi にもインストール

ラズパイで：

```bash
curl -fsSL https://tailscale.com/install.sh | sh
```

終わったら：

```bash
sudo tailscale up
```

すると、URLが出るので、**PCで開いてログイン**。

##### 5. ここまでで何が起きているか

この時点で：

* Windows
* Android
* Raspberry Pi

が **同じ仮想LAN** に入る

---

##### 6. ラズパイの固定IPを確認する

ラズパイで：

```bash
tailscale ip -4
```

例：

`100.64.12.34`

これがTaliscaleで表示される内容と一致するか確認する

---

##### 7. SSH接続

Windowsから：

```powershell
ssh pi@100.64.12.34
```

##### 8. 再起動時に自動接続

通常は自動で再接続されるが、念のため

```bash
sudo tailscale set --auto-update
```

---

### VSCodeでSSH接続したラズパイのターミナルを操作する方法

#### 1. VS Codeで拡張機能「Remote - SSH」をインストールする

#### 2. 接続

* 左下の「><」アイコン（リモート接続）からRemote-SSH: Connect to Host… を選択
* 以下を入力

```powershell
ssh ユーザー名@IPアドレス
```

* 接続後、VS Code 下部のステータスバーが「SSH: Raspberry Pi」表示になる

* Terminal → New Terminal を開くと、Pi のターミナルが利用可能

#### 3. 注意点

* 初回接続時は Pi 側に VS Code サーバが自動インストールされる。
* ターミナルは Pi のユーザ権限で動く（root操作は sudo）。

---

### Raspberry PiのIPアドレスをDiscordへ通知する

通知スクリプト本体は [`scripts/discord_ip.sh`](scripts/discord_ip.sh)、自動起動用のunitは [`deploy/systemd/discord-ip.service.example`](deploy/systemd/discord-ip.service.example) で管理する。Webhook URLはスクリプトやunitへ直接記載せず、Git管理外の`discord.env`から読み込む。

#### 1. Discord Webhookを作成する

Discordの対象チャンネルで「Server Settings」→「Integrations」→「Webhooks」を開き、Webhookを作成してURLをコピーする。

#### 2. 通知設定を作成する

```bash
cd ~/NSE2026
cp discord.env.example discord.env
nano discord.env
chmod 600 discord.env
```

`DISCORD_WEBHOOK_URL`にコピーしたURLを設定する。`discord.env`はGit管理外であり、コミットしない。

#### 3. 任意のタイミングで通知する

スクリプトを直接実行する場合:

```bash
cd ~/NSE2026
./scripts/discord_ip.sh
```

systemdの配置後は、同じスクリプトをservice経由でも実行できる。

```bash
sudo systemctl start discord-ip.service
sudo journalctl -u discord-ip.service -e
```

ネットワークまたはIPアドレスがまだ利用できない場合、スクリプトは30秒間隔で最大10回再試行する。

#### 4. OS起動時の通知を設定する

unitの配置と実機固有パスの編集は「本番運用（systemd）」の手順に従う。次回のOS起動からDiscord通知を実行する場合は、serviceをenableする。

```bash
sudo systemctl enable discord-ip.service
```

ミッションの自動起動を止める場合は`cansat.timer`だけを無効化する。`discord-ip.service`は独立しているため、その設定と手動実行には影響しない。

---

## 11. 参考資料

* Raspberry Pi公式ドキュメント: <https://www.raspberrypi.com/documentation/>
* Tailscale公式サイト: <https://tailscale.com/>
* Discordウェブフックドキュメント: <https://discord.com/developers/docs/resources/webhook>
* 設計メモ_TRC2026基板: <https://docs.google.com/document/d/1BoxN7ev75-qyxDMl1QDe3Ul_IqAFg0KLx-Su-Op7N-4/edit?usp=drive_link>
* 電子部品表_NSE2026: <https://docs.google.com/spreadsheets/d/1kxYLUrnnmavw1U_XV5uBSMUpxziN3frmR0-UFpshUV0/edit?gid=1327550036#gid=1327550036>
