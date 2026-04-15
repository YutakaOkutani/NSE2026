# テレメトリ仕様書

## 目的

このディレクトリは、CanSat のテレメトリを独立サブシステムとして再定義するための実装入口である。もともとの `runs/cam/cam_relay_*` はカメラフェーズのデバッグ用途だったが、今後は機体制御から分離された「送信・受信・表示・記録」専用のテレメトリとして育てる。

最優先は、Raspberry Pi Zero 2 W 上で安定して通常テレメトリを流し続けることである。映像は将来的に送るが、通常テレメトリとは別経路にし、映像の遅延や欠落が機体状態の観測を妨げない設計にする。

## 現行 `cam_relay_*` の問題点

- SBC 側が `HardwareManager`, `MotorManager`, `SensorManager`, `LedManager` を継承しており、テレメトリではなく機体制御の主体になっている。
- `Phase4Handler` から `Phase7Handler` まで直接持ち、フェーズ実行、モータスレッド、GPS/BNO、カメラ検出、TCP送信が同じプロセスに混在している。
- カメラ検出、bbox抽出、文字オーバーレイ、リサイズ、JPEGエンコード、base64化をSBC側で行っており、Zero 2 W には重い。
- 通常テレメトリと映像が同じTCPストリームの同じJSON payloadに混在しているため、映像が詰まると重要な状態情報も遅延する。
- JPEGをbase64でJSONに入れているため、帯域とCPU効率が悪い。
- 再接続はあるが、PC未接続時、欠損率、遅延、ログ、異常イベントの扱いが運用仕様として固定されていない。
- PC側UIがカメラデバッグ寄りで、運用中に見たいリンク状態、最新値、欠損、異常、ログ状態が主役になっていない。
- 将来 `main.py` と並走させるための疎結合な状態受け渡し口がない。

## 再定義された要件

### 必須要件

- テレメトリコードは機体制御を持たない。
- フェーズ遷移、モータ制御、カメラ検出判断を実行しない。
- 通常テレメトリを最優先する。
- PCが未接続でもSBC側は止まらない。
- Wi-Fi断やPC再起動後に自動で観測を再開できる。
- PC側に構造化ログを残す。
- SBC側にも小さなローカルspoolログを残す。
- Zero 2 W で現実に回る負荷に抑える。

### 将来要件

- `main.py` を動かしながらテレメトリを並走させる。
- `main.py` は制御主体、telemetry agent は状態購読者にする。
- 映像は通常テレメトリとは別ポート・別プロセスにする。
- 重い表示、解析、録画、地図化はPC側へ寄せる。
- systemd で自動起動できる構成へ進める。

## Phase A の範囲

Phase A は通信・記録・表示の最小安定版である。

- `telemetry_sbc.py`
  - UDPで通常テレメトリpacketを送信する。
  - デフォルトでは疑似データを送る。
  - `--source=file` でJSONファイルを状態入力として使える。
  - ローカルJSONL spoolを保存する。
- `telemetry_pc.py`
  - UDPで通常テレメトリpacketを受信する。
  - packetをJSONL保存する。
  - link age, rx Hz, packet loss, delay, 最新状態を表示する。
  - bad packet, link state change をイベントログに残す。

Phase A のデフォルトログ出力先:

- PC: `runs/telemetry/logs/telemetry_pc/<run_id>/telemetry.jsonl`
- PC: `runs/telemetry/logs/telemetry_pc/<run_id>/events.jsonl`
- SBC: `runs/telemetry/logs/telemetry_sbc/telemetry_sbc_<run_id>.jsonl`

Phase A では以下を行わない。

- `main.py` との統合。
- センサの直接読み取り。
- モータ操作。
- フェーズ実行。
- 映像送信。
- PCからSBCへの制御コマンド。

## 推奨アーキテクチャ

最終的な構成は次を目指す。

```text
[SBC: main.py / mission process]
        |
        | local state publish
        | Unix datagram socket or localhost UDP
        v
[SBC: telemetry agent]
        |                         \
        | UDP telemetry             \ separate video stream
        v                           v
[PC: ground station]            [PC: video viewer/recorder]
```

`main.py` は機体制御に集中する。telemetry agent は `CanSatState.snapshot()` 相当の状態を受け取り、PCに送るだけにする。telemetry agent が落ちても、`main.py` は継続できる必要がある。

## 通信仕様

### 通常テレメトリ

- Transport: UDP
- Default port: `5001/udp`
- Encoding: JSON
- 1 datagram = 1 packet
- Schema: `cansat.telemetry.v1`
- Default rate: 5 Hz
- Recommended max payload: 1200 bytes

UDPを使う理由は、通常テレメトリでは古いpacketの再送よりも新しい状態が届くことの方が重要だからである。欠落はPC側で `seq` から推定し、画面とログに出す。

### 映像通信

Phase A では未実装。将来は以下を目指す。

- Transport: UDPまたはRTP相当
- Default port: `5002/udp`
- 320x240, 3-5 fps から開始
- H.264低ビットレート、またはMJPEG
- 通常テレメトリpacketへ画像本体を混ぜない
- テレメトリ側には `video: off/on/lost` のような状態だけを載せる

## Packet v1

代表例:

```json
{
  "schema": "cansat.telemetry.v1",
  "type": "state",
  "seq": 12345,
  "boot_ms": 456789,
  "sent_unix": 1710000000.123,
  "mission": {
    "phase": 4,
    "mode": "RUNNING",
    "reason": ""
  },
  "gps": {
    "lat": 35.0,
    "lng": 139.0,
    "fix": 1,
    "sats": 8,
    "hdop": 1.2,
    "heading_deg": 123.4,
    "heading_valid": true,
    "speed_mps": 1.5
  },
  "imu": {
    "acc": [0.0, 0.0, 9.8],
    "gyro": [0.0, 0.0, 0.0],
    "mag": [0.0, 0.0, 0.0],
    "angle_deg": 90.0,
    "angle_valid": true,
    "fall": 0.0
  },
  "baro": {
    "alt_m": 12.3,
    "pressure_hpa": 1010.1
  },
  "nav": {
    "distance_m": 21.5,
    "azimuth_deg": 85.0,
    "direction": 0.52
  },
  "actuator": {
    "motor_left": 0.3,
    "motor_right": 0.3,
    "last_cmd_age_ms": 120
  },
  "health": {
    "cpu_temp_c": 55.0,
    "state_age_ms": 80,
    "errors": [],
    "video": "off"
  }
}
```

## UI仕様

PC側UIは、グラフより先に運用状態の即時判断を優先する。

- LINK
  - `OK`, `DEGRADED`, `LOST`, `WAITING`
  - link age
  - rx Hz
  - packet loss
  - last seq
  - delay
- MISSION
  - phase
  - mode
  - reason
  - boot time
- GPS
  - fix
  - lat/lng
  - sats
  - hdop
  - speed
  - heading
- IMU
  - angle
  - acc
  - gyro
  - fall
- NAV / ACTUATOR
  - distance
  - azimuth
  - direction
  - motor command
  - command age
- HEALTH
  - CPU temp
  - state age
  - payload size
  - video status
  - errors
- LOG
  - run directory
  - bad packet count

## 異常時動作

- PC未起動:
  - SBCはUDP送信を続ける。
  - SBCはローカルspoolログを残す。
- Wi-Fi断:
  - SBCは送信処理を継続する。
  - PCは `LOST` に遷移し、イベントログへ記録する。
- packet欠落:
  - PCは `seq` のgapからloss率を推定する。
- malformed packet:
  - PCはpacketを捨て、`events.jsonl` に理由を残す。
- payload過大:
  - SBCは `health.errors` に `payload_over_limit` を入れる。
- state source不良:
  - `--source=file` では `state_file_missing` や `state_file_json_error` を `health.errors` に載せる。

## 段階的移行計画

1. Phase A: UDP通常テレメトリ、PCログ、軽量UIを固める。
2. Phase B: `main.py` からlocal publishし、telemetry agentが購読する。
3. Phase C: 映像を別ポート・別プロセスで追加する。
4. Phase D: PC側UIを強化し、イベント、地図、グラフ、録画管理を加える。
5. Phase E: systemd、自動起動、ログローテーション、長時間試験へ進める。

## 本番運用までのロードマップ

### 最小実装

- 5Hz UDP送信。
- PC受信。
- JSONL保存。
- link age / rx Hz / loss 表示。

### `main.py` 連携

- `CanSatState.snapshot()` をlocal publishする。
- `last_motor_command`, `mission_end_reason`, `phase7_arrival_reason` を含める。
- telemetry agent はlocal stateを購読してPCへ転送する。

### 映像分離

- 画像本体を通常テレメトリpacketへ混ぜない。
- 低解像度・低fpsから開始する。
- 映像が落ちても通常テレメトリを守る。

### 運用強化

- systemd service化。
- ログローテーション。
- CPU温度、ディスク残量、state freshnessをhealthに含める。
- 30分以上の連続試験。
- PC再起動、Wi-Fi断、映像ON/OFF試験。

## 成功条件

Phase Aの成功条件:

- PCを途中で落としてもSBC側が止まらない。
- PCを再起動したら自動で受信できる。
- 10分以上動かしてlink age, rx Hz, packet lossが見える。
- PC側ログからpacket時系列を復元できる。
- SBC側処理が軽く、映像や制御を持たない。

本番前の成功条件:

- `main.py` 実行中に通常テレメトリが並走できる。
- Wi-Fi不調で制御周期が乱れない。
- 映像をONにしても通常テレメトリが遅延しない。
- 異常時刻と理由をログから追える。
