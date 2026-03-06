# go2-gait-runner 実行マニュアル

articara の歩容生成（`quadruped-gait`）で確認した歩容を、articara 本体（GUI）を使わず
`quadruped-gait` + `misarta` + `unitree-sdk-rs` で **実機 Unitree Go2** 上で歩かせるための
手順書。設計の背景・依存関係・チューニング結果は
[go2_realrobot_gait_plan.md](../../articara/doc/go2_realrobot_gait_plan.md) を参照。

> ⚠️ **安全第一**: 低レベル（rt/lowcmd）でモータを直接駆動する。必ず周囲を空け、脚を吊れる
> 状態（昇降台・吊り紐など）から始め、緊急停止（電源/コントローラ）を手元に置くこと。

---

## 1. 事前準備（ハードウェア）

1. **電源**: バッテリ運用でも AC アダプタ運用でも可。残量が減ったら AC へ切り替える。
2. **ネットワーク（有線）**: PC の `eth0` を Go2 と同じサブネットに設定する。
   - robot: `192.168.123.161`（固定）
   - PC eth0: `192.168.123.99/24`
   ```bash
   sudo ip addr add 192.168.123.99/24 dev eth0
   sudo ip link set eth0 up
   ping -c2 192.168.123.161        # 疎通確認
   ```
   （恒久化するなら NetworkManager で eth0 プロファイルに static IP を登録する。）
3. **sport_mode を OFF にする**: 出荷時の高レベル運動制御（sport_mode）が動いていると
   rt/lowcmd と競合し、関節が発振する。低レベル制御の前に必ず解放する。
   `run`/`diag` は**起動時に自動で解放する**ので通常は手動操作は不要（`--no-release`
   で抑止できる）。手動で操作したい場合は go2-gait-runner のサブコマンドを使う:
   ```bash
   ./target/release/go2-gait-runner release   eth0   # sport_mode OFF（脱力）
   ./target/release/go2-gait-runner restore   eth0   # sport_mode ON（立位を取る）
   ./target/release/go2-gait-runner checkmode eth0   # 現在のモードを確認（read-only）
   ```
   これは `motion_switcher` RPC を **純 Rust（`unitree-rpc` クレート）で直接叩く**実装で、
   従来の C++ ヘルパ `go2_motion_ctrl` と等価。C++ 側も引き続き使用可:
   ```bash
   ~/work/keel/unitree_sdk2/build/bin/go2_motion_ctrl release eth0
   ```

詳細なブリングアップは [unitree-sdk-rs/doc/go2-bringup.md](../../unitree-sdk-rs/doc/go2-bringup.md) を参照。

---

## 2. ビルド

```bash
cd go2-gait-runner
# cyclonedds-sys が libddsc.so を解決できるよう、arch 別の CycloneDDS を指す。
# Unitree SDK2 同梱の thirdparty が使える（x86_64 / aarch64 両方同梱）:
export UNITREE_SDK2_ROOT=/path/to/unitree_sdk2     # thirdparty/{include,lib/<arch>}
#   または: export CYCLONEDDS_HOME=/path/to/cyclonedds-install
cargo build            # デバッグ（articara・unitree-sdk-rs を GitHub から取得）
# cargo build --release  # 実機運用は release 推奨
```

- 依存（`quadruped-gait`/`misarta`/`unitree-go2`/`unitree-rpc`）は **GitHub から git 依存**で
  自動取得される（sibling clone 不要、`README.md` 参照）。
- `cyclonedds-sys` は `libddsc.so` を vendor していないため、ビルド時に
  `UNITREE_SDK2_ROOT/thirdparty/lib/<arch>`（または `CYCLONEDDS_HOME/lib/<arch>`）から解決する。
  FFI バインディングは x86_64 / aarch64 とも commit 済み（新 arch のみ
  `unitree-sdk-rs/tools/regen-bindings.sh` で生成、libclang 必須）。
- 実行時も同じ lib ディレクトリを `LD_LIBRARY_PATH` に設定する。`cargo run` は自動設定するが、
  **ビルド済みバイナリを直接実行する場合は手動で設定**すること:
  ```bash
  export LD_LIBRARY_PATH=$UNITREE_SDK2_ROOT/thirdparty/lib/x86_64   # arch に合わせる
  ```
  設定し忘れると次のエラーで起動に失敗する:
  ```text
  error while loading shared libraries: libddsc.so.0: cannot open shared object file
  ```
  毎回打ちたくない場合は `~/.bashrc` に追記して恒久化しておく（以降のシェルで自動設定）:
  ```bash
  echo 'export LD_LIBRARY_PATH=/home/takara/cyclonedds-install/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
  source ~/.bashrc
  ```

---

## 3. オフライン検証（ロボット不要）

実機に送る前に、歩容がリミット内かを必ず確認する。

```bash
# (a) 関節リミットチェック: 全 tick が Go2 の関節範囲内かを assert
cargo run -p go2-gait-runner -- dump

# (b) 歩容の意図確認: 前進量・足先スイープ・足上げ量・重心配分FF を表示
cargo run -p go2-gait-runner -- intent --vx 0.02 --cycle 2.5 --four-support 0.9 --swing 0.04
```

`intent` は重心（CoM）位置と、各支持脚へ配分される重量フィードフォワード（FF）トルクも表示する。

---

## 4. 実機実行（安全ラダー）

`run`（= `diag` エイリアス）は次の4フェーズを 500Hz で実行する。各フェーズの前に状態を
確認しながら進める。

| フェーズ | 内容 | 時間 |
|---|---|---|
| A | 開始姿勢 → 歩容 nominal stance へ ramp（kp 漸増） | `RAMP_SECS` 2.0s（固定） |
| B | 在地 vx=0（位相凍結、立位保持） | `--inplace`（既定 3s） |
| C | 前進（vx>0 のときのみ）: 加速 → 一定 → 減速 | 1.5 + `--forward`(既定4s) + 1.5s |
| D | 伏せ姿勢へ fold（安全終了） | `FOLD_SECS` 2.0 + 0.5s（固定） |

> 合計（既定・vx>0）≈ 14.5s / vx=0 のときは C をスキップして ≈ 7.5s。

### 手順

```bash
# 0) sport_mode OFF（再掲）
~/work/keel/unitree_sdk2/build/bin/go2_motion_ctrl release eth0

# 1) まずは在地のみ（vx=0）で送信パス全体を立位静止で検証
cargo run -p go2-gait-runner -- run eth0 --kp 200 --kd 6 --ff

# 2) 問題なければ微速前進（動作確認済みの推奨設定）
cargo run -p go2-gait-runner -- run eth0 \
    --vx 0.02 --cycle 2.5 --four-support 0.9 --swing 0.04 \
    --kp 200 --kd 6 --ff
```

実行中は commanded vs measured のトレースが流れ、終了時に**追従誤差＋胴体傾きのサマリ**が出る。
途中で異常があれば即座に電源/コントローラで停止する。終了は自動で伏せ姿勢へ折り畳む。

### 動作確認済みの推奨設定（滑りやすいフローリング）

```
--vx 0.02 --cycle 2.5 --four-support 0.9 --swing 0.04 --kp 200 --kd 6 --ff
```
- vx=0.02 m/s で滑る床でも素直に前進、胴体ロール最大 ~1.0°。
- **滑る床では「遅いほど確実」**。vx を上げる/cycle を速くすると滑って前後にブレる。

---

## 5. テレメトリの CSV 保存

`--csv PATH` を付けると、記録対象（フェーズ B+C）の**全テレメトリ**を 500Hz・1行/tick で保存する。

```bash
cargo run -p go2-gait-runner -- run eth0 \
    --vx 0.02 --cycle 2.5 --four-support 0.9 --kp 200 --kd 6 --ff \
    --csv /tmp/go2_telemetry.csv
```

### CSV の列（70列）
| 区分 | 列 |
|---|---|
| 時刻/フェーズ | `t_s`（先頭・単調増加）, `phase`（B/C） |
| IMU | `roll, pitch, yaw, gyro_x/y/z, acc_x/y/z, quat_w/x/y/z, imu_temp` |
| 電源 | `power_v, power_a` |
| 接地力 | `foot0..foot3` |
| 各関節×12 | `<関節>_cmd`(指令角), `_q`(実測角), `_dq`(角速度), `_tau`(推定トルク) |

関節順は Go2 モータ順（`FR/FL/RR/RL × hip/thigh/calf`）。値は IMU の `rpy` がロール/ピッチ/ヨー。
解析の目安: 滑り → `foot*` と `*_dq`、揺れ → `roll/pitch/gyro`、脚の負荷 → `*_tau`。

---

## 6. CLI リファレンス

`go2-gait-runner -h` で全一覧が出る。

```
USAGE:
  go2-gait-runner <mode> [<iface>] [flags]

MODES:
  dump            オフライン: 歩容が Go2 関節リミット内かを assert
  intent          オフライン: 前進量・足先スイープ・足上げを定量化
  run    <iface>  実機: ramp→在地→前進→fold。状態を読み戻しサマリ表示
  diag   <iface>  run のエイリアス（同一動作）

FLAGS（すべて任意。run/diag は第1 positional が <iface>）:
  --misa PATH       モデル .misa（既定 models/unitree_go2/go2.misa）
  --vx V            前進速度 m/s（run/diag 既定 0.0、intent 0.05）
  --inplace S       在地フェーズ秒（既定 3）            [run/diag]
  --forward S       前進保持フェーズ秒（既定 4）        [run/diag]
  --kp K            位置ゲイン（既定 60）               [run/diag]
  --kd K            減衰ゲイン（既定 5）                [run/diag]
  --swing H         足上げ高さ m（既定 0.04）
  --cycle S         歩行周期 s（既定: crawl プリセット）
  --four-support F  4脚支持比 0..1（既定: crawl プリセット）
  --max-swing-speed V  遊脚足先ピーク速度の上限 m/s（既定 3.0、0 で無効）。
                    4脚支持比を上げると遊脚窓が縮み足先速度が爆発(≈8v/(1−α))
                    して胴体が揺れる。これを前進速度の自動制限で抑える。
  --ff              重量フィードフォワード有効          [run/diag]
  --ff-scale S      FF を実機質量へスケール（既定 1.0、実機 ~1.73）  [run/diag]
  --csv PATH        全テレメトリ CSV を保存             [run/diag]
  -h, --help        ヘルプ表示
```

---

## 7. トラブルシュート

| 症状 | 原因 / 対処 |
|---|---|
| `libddsc.so.0 not found`（直接実行時） | `export LD_LIBRARY_PATH=/home/takara/cyclonedds-install/lib`。`cargo run` なら自動。 |
| `timeout waiting for LowState` | iface 名・`192.168.123.x` の IP・ケーブルを確認。`ping 192.168.123.161`。 |
| ロボットが指令に反応しない / 競合 | sport_mode が ON のまま。`go2_motion_ctrl release eth0` を実行。 |
| 脚が荷重で沈む（calf 追従誤差大） | `--ff` を付ける。足りなければ `--kp` を上げる。 |
| 前後にブレる/後退する | 床が滑っている。`--vx` を下げる（0.02 程度）、`--swing` を確保（引きずり防止）。 |
| 遊脚期に胴体が揺れる | **`--four-support` を上げ過ぎると遊脚窓が縮み、足先指令速度が `≈8·v/(1−α)` で爆発してアクチュエータが追従できず揺れる**（α=0.9, v=0.05 でピーク ~4 m/s）。対策は (1) `--max-swing-speed`（既定 3.0）を揺れが消える値まで下げる＝前進速度が自動で落ちて滑らかになる、(2) `--four-support` を下げる、(3) `--vx` を下げる。**`--cycle` を伸ばしても直らない**（歩幅も周期に比例し足先速度は不変）。従来仕様に戻すなら `--max-swing-speed 0`。 |

---

## 8. 関連ドキュメント
- [go2_realrobot_gait_plan.md](../../articara/doc/go2_realrobot_gait_plan.md) — 設計・依存関係・チューニング結果
- [unitree-sdk-rs/doc/go2-bringup.md](../../unitree-sdk-rs/doc/go2-bringup.md) — Go2 低レベル SDK ブリングアップ
