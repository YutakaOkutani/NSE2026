# runs/telemetry 設計・デバッグ指針

`runs/telemetry/` は、CanSat の状態を観測し続けるための独立テレメトリ層である。ここでは「機体を動かすこと」ではなく、「機体が今どうなっているかを軽く、安定して、後から追える形で外へ出すこと」を価値とする。

## 目的

- 通常テレメトリを安定してPCへ届ける。
- PC側で見やすく表示し、必ず構造化ログに残す。
- Wi-Fi断、PC未起動、PC再起動を前提にする。
- 将来 `main.py` と並走できる疎結合な構成へ進める。
- Zero 2 W で現実に回る軽さを守る。

## 重要原則

- テレメトリは機体制御を持たない。
- フェーズ遷移、モータ制御、停止判断、カメラ検出判断をここに入れない。
- 通常テレメトリと映像を混ぜない。
- 古いpacketの再送より、新しい状態の到着を優先する。
- 重い処理はPC側へ寄せる。
- ログはデバッグ補助ではなく、改善の根拠として扱う。

## 現行カメラ中継からの分岐理由

`runs/cam/relay_*` はカメラフェーズのデバッグには有効だったが、純粋テレメトリとしては責務が大きすぎる。SBC側でフェーズハンドラ、モータスレッド、センサ読み取り、カメラ検出、画像加工、TCP送信を同時に扱うため、将来 `main.py` と並走する構成では衝突しやすい。

このため、`runs/telemetry/` は既存コードを直接改造せず、新しいサブシステムとして分岐する。

## Phase A のデバッグ観点

Phase Aでは、次の項目を必ず見る。

- PC側 `status`
  - `OK`, `DEGRADED`, `LOST`, `WAITING`
- `link age`
  - 最後にpacketを受け取ってからの秒数。
- `rx Hz`
  - 実際の受信周期。
- `loss`
  - `seq` gapから推定したpacket loss。
- `delay`
  - `sent_unix` と受信時刻の差。
- `bad_packets`
  - JSON破損、schema不一致、type不一致など。
- `health.errors`
  - SBC側が自分で検出した異常。
- `payload_bytes`
  - 1200 bytesを大きく超えないか。

## ログ方針

PC側ログ:

- `runs/telemetry/logs/telemetry_pc/<run_id>/telemetry.jsonl`
  - 受信したpacket本体。
  - 受信時刻。
  - 送信元アドレス。
- `runs/telemetry/logs/telemetry_pc/<run_id>/events.jsonl`
  - receiver start/stop。
  - link state change。
  - bad packet。
  - invalid packet。

SBC側ログ:

- `runs/telemetry/logs/telemetry_sbc/telemetry_sbc_<run_id>.jsonl`
  - 送信packetの間引きspool。
  - `--log-every` で頻度を調整する。
  - ログ肥大化を避けるためrotationを行う。

## 異常の見方

### PC側が `WAITING`

- まだ1packetも受信していない。
- SBC側の `--pc-host` がPCのIPになっているか確認する。
- PCファイアウォール、同一Wi-Fi、UDP 5001を確認する。

### PC側が `LOST`

- 以前は受信していたが3秒以上途絶えている。
- Wi-Fi断、SBC停止、送信先IP変更を疑う。
- `events.jsonl` の `link_state_change` を見る。

### lossが高い

- Wi-Fi品質が悪い。
- payloadが大きすぎる。
- tx Hzが高すぎる。
- PC側の処理が詰まっている。

### delayが大きい

- PC/SBCの時刻がずれている可能性がある。
- Wi-Fi遅延の可能性もある。
- 絶対値を過信せず、急な増加傾向を見る。

### payload_over_limit

- packetが1200 bytesを超えている。
- 項目を削るか、送信頻度の低いdiagnostic packetへ分ける。
- 映像や大きな文字列を通常テレメトリに入れてはいけない。

## 変更時のルール

- packet schemaを変える場合は `SPEC.md` を更新する。
- フィールドを追加する場合は、単位、異常時の値、PC側表示の要否を決める。
- SBC側に重い依存を増やさない。
- `telemetry_sbc.py` がセンサデバイスやモータデバイスを直接握る変更は避ける。
- PC側UIの改善でSBC側payloadを膨らませない。
- 映像実装は通常テレメトリと別ファイル・別ポートにする。

## 避けたい状態

- テレメトリが止まると `main.py` も止まる。
- 映像が詰まると通常テレメトリも遅れる。
- PCがいないとSBC側で例外が出続ける。
- ログが残らず、実験後に何が起きたか分からない。
- 便利だからという理由でモータ停止やフェーズ遷移をテレメトリ側へ入れる。
- packetに巨大なdebug文字列や画像を入れる。

## 次のステップ

1. Phase Aを10分以上動かし、PC再起動とSBC継続送信を確認する。
2. `--source=file` を使い、外部状態JSONから送信できることを確認する。
3. `main.py` 側にlocal state publisherを追加する。
4. telemetry agentを `main.py` の状態購読者にする。
5. 映像を別ポートで追加する。
6. systemd service化と長時間試験を行う。
