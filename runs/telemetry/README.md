# CanSat Telemetry Phase A

`runs/telemetry/` は、既存の `runs/cam/relay_*` から分岐した純粋なテレメトリ実装の入口である。

Phase A では `main.py` との接続や映像送信はまだ行わず、通常テレメトリの通信・受信・表示・記録を先に固める。

## Run

PC 側:

```bash
python3 runs/telemetry/telemetry_pc.py --host 0.0.0.0 --port 5001
```

SBC 側:

```bash
python3 runs/telemetry/telemetry_sbc.py --pc-host <PC_IP> --pc-port 5001 --tx-hz 5
```

同じPC上で通信テストする場合:

```bash
python3 runs/telemetry/telemetry_pc.py
python3 runs/telemetry/telemetry_sbc.py --pc-host 127.0.0.1
```

## Outputs

PC:

- `runs/telemetry/logs/telemetry_pc/<run_id>/telemetry.jsonl`
- `runs/telemetry/logs/telemetry_pc/<run_id>/events.jsonl`

SBC:

- `runs/telemetry/logs/telemetry_sbc/telemetry_sbc_<run_id>.jsonl`

## Phase A Scope

- UDP 5001 で通常テレメトリを送る
- 1 packet = 1 JSON datagram
- PC 側で packet loss, link age, rx Hz を表示する
- PC 側で JSONL ログを保存する
- SBC 側処理は軽量に保つ
- 映像、機体制御、フェーズ実行、モータ操作は扱わない

詳細は `SPEC.md` と `DEBUG_POLICY.md` を参照する。
