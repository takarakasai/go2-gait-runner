# go2-gait-runner 改善点

実機 Go2 を 500 Hz で低レベル駆動する安全クリティカルな CLI として、`src/main.rs`
をレビューした際の改善点を優先度順にまとめる。行番号は記載時点のもの。

## 1. 安全性（実機駆動なので最優先）

### 1.1 通信ウォッチドッグが無い
- 制御ループ中の `reader.poll()`（`src/main.rs:1596`）は非ブロッキングで、
  `LowState` が来なくても `None` のまま走り続ける。
- 走行中にケーブルが抜けても leveling / observer は古い値のまま gait をオープン
  ループで出し続け、**異常停止しない**。
- 対策: 「N tick 連続で新しい state が来なければ damping / fold へ移行」する
  ウォッチドッグを追加する。

### 1.2 Ctrl-C / シグナルハンドラが無い
- 走行中に SIGINT を受けるとプロセスが即死し、ロボットは最後のコマンドのまま
  放置される（fold もダンピングもしない）。
- 対策: SIGINT で安全姿勢（kp を下げてダンピング、または fold）へ落とすハンドラ
  を実装する。ハードウェアでは必須級。

### 1.3 実行時の関節リミットクランプが無い
- リミット検査は `dump` モードだけ（`src/main.rs:1112`）。
- 実走行では `--level` 補正・FF・sway・stance 上書きが乗るので、`dump` で検証した
  軌道とは別物になり得る。
- 対策: `emit` 直前で `q` を `LIMITS` にクランプする防御を入れる。

### 1.4 開始姿勢の妥当性チェックが無い
- `wait_for_start_pose`（`src/main.rs:1140`）が捕えた姿勢が standing から大きく
  外れていても、2 秒で stance へ kp を上げながら ramp する。寝ている状態から始め
  ると急激な動きになり得る。
- 対策: 開始姿勢が想定範囲内かを確認してから ramp に入る。

## 2. リアルタイム性（Raspberry Pi 上の 500 Hz）

### 2.1 制御ループ内でブロッキング I/O
- `--viz` の Zenoh put（`src/main.rs:1566` → `publish` 内の `.wait()`
  `src/main.rs:120`）と CSV 書き込みが 500 Hz ループ内で同期実行される。
- 「エラーは無視」しているのは*エラー*であって*レイテンシ*ではない。ネットワーク
  put や fs 書き込みのブロックは tick のデッドラインを乱す。
- 対策: viz / CSV は専用スレッド + チャネル（満杯ならドロップ）へ逃がす。記録は
  B+C で有限なので、メモリにバッファして走行後にまとめて書く手もある。

### 2.2 デッドライン超過の検知・報告が無い
- 絶対時刻ペーシング自体は良い（`src/main.rs:1327-1330`、ドリフトしない）が、
  超過した tick を数えていない。
- Pi の非 RT カーネル + `thread::sleep` ではジッタが出るので、超過回数 / 最大
  ジッタを集計して summary に出すと調律の判断材料になる。
- 対策: RT 優先度（`SCHED_FIFO`）と `mlockall` の検討も含める。

## 3. 具体的なバグ

### 3.1 起動ログが常に "LinearCrawl" 固定
- 起動サマリ（`src/main.rs:1291`）は `--gait mpc` を指定しても
  `"go2-gait-runner: LinearCrawl ..."` と出る。`tune.gait_mode` を反映すべき。
- help 冒頭文（`src/main.rs:294`）も "drive ... LinearCrawl" のままで、複数 gait
  対応とドリフトしている。

### 3.2 `--gait` を二重パース・挙動不一致
- `src/main.rs:383-405` で `gait_mode` を作った後、`gait_type` ブロックで
  `cli.str("gait").and_then(parse_gait_mode)` を再度呼んでいる。
- 片方は不正値で `exit(1)`、もう片方は `unwrap_or(LinearCrawl)` で**黙って無視**
  と挙動が不一致。
- 対策: 先に mode を 1 回解決して使い回す。

## 4. 構造・保守性

### 4.1 引数が爆発している
- `record` クロージャは **16 引数**（`src/main.rs:1394-1412`）、`run_hardware` は
  13 引数で `#[allow(clippy::too_many_arguments)]`。
- 対策: アキュムレータ群（err_sum/max, roll_max, yaw_* など）は `Recorder` 構造体
  に `&mut self` でまとめ、引数群は `RunConfig` にまとめる。

### 4.2 モーター順マッピングが 3 重定義
- FR=0 / FL=3 / RR=6 / RL=9 のテーブルが `go2_motor_index`（`src/main.rs:164`）、
  `leg_base_motor`（`src/main.rs:755`）、`leg_q_dq_ik`（`src/main.rs:1179`）の
  3 箇所にある。`gait_slot` / `leg_slot` も同じ FL/FR/RL/RR 写像の重複。
- 安全クリティカルな添字なので**単一の真実源**に集約すべき（片方を直し忘れると
  実機で破綻）。`run_dump` 内の変換（`src/main.rs:1100-1109`）も `output_to_go2`
  の再実装なので統合できる。

### 4.3 手書き CLI パーサ
- `parse_cli` + `BOOL_FLAGS` 手管理 + 手書き help は、フラグ追加のたびにドリフト
  する。
- 対策: `clap`（derive）にすればバリデーション・help 自動生成・typo 検出が手に
  入る。`viz_cfg` も intent と run でほぼ同一コードを 2 回書いている
  （`src/main.rs:422` と `src/main.rs:464`）ので `VizCfg::from_cli` に抽出。

### 4.4 データとログが両方 stderr
- `dump` / `intent` の CSV 風出力も summary も全部 `eprintln!`。
- 対策: 機械可読データは stdout、ログは stderr（または `tracing`）に分ける。

### 4.5 CSV の明示 flush が無い
- `BufWriter` は Drop でフラッシュするがエラーを握りつぶす。
- 対策: 末尾で `flush()?` して書き込み失敗を表面化させる。

## 5. テストが皆無

- `go2_motor_index` / `gait_slot` / `leg_slot` / `leg_base_motor` /
  `output_to_go2` は純粋関数で、間違えると実機事故になる添字・符号ロジック。
- 対策: 「3 つのテーブルが一致する」「`go2_motor_index("FL_thigh")==4`」等の
  ユニットテストを入れれば、ハードに繋ぐ前に table のタイポを捕まえられる。
  現状はタイポが実機でしか露見しない。

## 6. 機能

- **`vy` / `wz`（旋回）が未対応**: `VelocityCmd` は対応しているのに
  `src/main.rs:1628` 等で `vy:0, wz:0` 固定。前進バーストのみ。
- **対話 / 連続運転モードが無い**: ramp→inplace→forward→fold の固定シーケンス
  1 発のみ。`src/main.rs:1612` のコメント通り外部 Twist（stdin / zenoh / gamepad）
  入力を繋ぐと「runner」として実用度が上がる。
- **`diag` は `run` の完全エイリアス**: 機能差が無いなら統合 or 削除。

## 優先度まとめ

- **最優先**: 1（ウォッチドッグ・SIGINT フォールド・実行時クランプ）— 実機破損に
  直結。
- **安価で効果大**: 3.1（ログ固定バグ）— すぐ直せる。
- **中期**: 4.2 添字マッピング統合 + 5 ユニットテスト — 安全性と保守性の両取り。
