# Go2 起動時の自動立ち上がりを止める — 調査メモ

調査日: 2026-06-16 / 対象機: `192.168.123.161`（ホスト名 `Unitree`、root でSSH可）

## 背景

電源投入時に Go2 が勝手に立ち上がるのを止めたい。これはオンボードLinux上の
サービスが行っており、root でログインして調査・無効化できる。

- ロボット内部は Linux（`5.10.176-rt86+`, aarch64, PREEMPT_RT）。
- SSH: `root@192.168.123.161`（公開鍵認証を登録済み。手元PC eth0 は `192.168.123.99/24`）。

## 立ち上がりの正体

起動時の立ち上がりの本体は **`/unitree/module/sport_mode/Legged_sport`**（`sport_mode` サービス）。
電源投入直後ロボットは伏せた状態で、`sport_mode` が起動して `Legged_sport` が走ると立ち姿勢を取る。

```
root ... /unitree/module/sport_mode/Legged_sport      # ← これが立たせる
root ... /unitree/module/motion_switcher/motion_switcher_service
root ... /unitree/module/robot_state/robot_state_service
root ... /unitree/module/basic_service/basic_service
root ... /unitree/module/master_service/master_service # ← 各モジュールの管理者
```

## 起動管理の仕組み

- `master_service`（`/etc/init.d/master_service` = LSB initスクリプト経由で起動）が
  各モジュールの起動/停止/自動起動を管理する。systemd 側に unitree 用ユニットは無い。
- 操作CLIは **`/unitree/sbin/mscli`**。

```
mscli listservice            # 全サービスの状態と enable(自動起動) 一覧
mscli getservice  [name]     # 個別状態
mscli getenable   [name]     # 自動起動の有無
mscli startservice [name]    # 起動
mscli stopservice  [name]    # 停止
mscli restartservice [name]
mscli reloadservice [name]   # サービス定義ファイル(JSON)を更新したら再読込
mscli saveservice  [name][config]
mscli removeservice [name]
```

- サービス定義は平文JSON: `/unitree/etc/master_service/service/<name>`。
  中身は `Start` / `Stop` / `Status` の各 `Cmd`（`start-stop-daemon` 呼び出し）のみ。
- 起動計画・依存・**enableフラグ**は暗号化ファイル（先頭マジック `FMX...`）側にある:
  `/unitree/etc/master_service/{plan,init,once,manual,forbid,conflict,protect,path}`。
  → 平文で直接編集できない。`mscli` に `setenable` も無い。

### `mscli listservice` 抜粋（調査時点）

`enable:1` = 起動時に自動起動 / `enable:0` = 自動起動しない。

| service | enable | 備考 |
|---|---|---|
| **sport_mode** | **1** | ← Legged_sport を起動。立ち上がりの原因 |
| motion_switcher | 1 | モード切替（release/restore はここ） |
| robot_state | 1 | |
| basic_service | 1 | |
| advanced_sport | 0 | 自動起動しない |
| ai_sport | 0 | 自動起動しない |
| 4gcm | 0 | 自動起動しない |
| (他 audio_hub, vui_service, webrtc_* 等は enable:1) | | |

`advanced_sport`/`ai_sport`（enable:0）のサービスJSONは sport_mode と同じ構造で、
**enableの差はJSONには無く暗号化plan側にある**ことを確認済み。

### sport_mode のサービス定義（`/unitree/etc/master_service/service/sport_mode`）

```json
{
  "Start":  { "Cmd": "/unitree/sbin/start-stop-daemon --start --background --output=/tmp/sport_mode.LOG --cpuset=4,5 --make-pidfile --pidfile=/unitree/var/run/Legged_sport.pid --exec /unitree/module/sport_mode/Legged_sport", "ExpectCode": [0] },
  "Stop":   { "Cmd": "/unitree/sbin/start-stop-daemon --stop  --pidfile=/unitree/var/run/Legged_sport.pid --exec /unitree/module/sport_mode/Legged_sport", "ExpectCode": [0,1] },
  "Status": { "Cmd": "/unitree/sbin/start-stop-daemon --status --pidfile=/unitree/var/run/Legged_sport.pid --exec /unitree/module/sport_mode/Legged_sport", "AliveCode": [0], "DeadCode": [1,3] }
}
```

## 自動立ち上がりを止める方法（候補・いずれも可逆）

> 安全上の注意: ロボットが**立っている最中に sport_mode を止めると脱力して崩れ落ちる**。
> 電源投入直後は伏せた状態なので、設定変更してから**再起動して確認**するのが最も安全。
> 設定変更後は、自動立ち上がりだけでなく**アプリ／リモコンでの通常動作も無効**になる
> （低レベル制御 rt/lowcmd 主体の開発にはむしろ好都合）。元に戻せば通常動作に復帰。

### 案A: sport_mode の Start を無効化（推奨）

`service/sport_mode` をバックアップし、Start の `Cmd` を no-op（`/bin/true`）に書き換え、
`reloadservice` する。次回起動から Legged_sport が走らず立たない。master_service の
仕組み内で完結し、JSONを戻して reload すれば復旧。

```bash
# 無効化
cp /unitree/etc/master_service/service/sport_mode /root/sport_mode.json.bak
# Start.Cmd を "/bin/true" に書き換え（Stop/Status はそのまま）
mscli reloadservice sport_mode
mscli stopservice  sport_mode   # ← 今動いているものも止める（伏せ状態で）

# 復旧
cp /root/sport_mode.json.bak /unitree/etc/master_service/service/sport_mode
mscli reloadservice sport_mode
mscli startservice  sport_mode
```

### 案B: Legged_sport バイナリをリネーム

設定に触れず、起動コマンドの `--exec` を失敗させる。最も単純で `mv` で即復旧。
start-stop-daemon が起動失敗のログを残すだけ（無害）。

```bash
# 無効化
mv /unitree/module/sport_mode/Legged_sport /unitree/module/sport_mode/Legged_sport.disabled
mscli stopservice sport_mode   # 伏せ状態で
# 復旧
mv /unitree/module/sport_mode/Legged_sport.disabled /unitree/module/sport_mode/Legged_sport
mscli startservice sport_mode
```

### 案C: その都度手動停止（永続しない）

```bash
mscli stopservice sport_mode   # 再起動で復活。立っている時は崩れるので注意
```

## go2-gait-runner との関係（参考）

- 本ランナーの `release`（`motion_release`）は motion_switcher RPC で sport_mode を
  「解除」して脱力させるもの。**実行はロボット起動後**なので、電源投入直後の
  最初の立ち上がり自体は防げない。上記A/Bはそれより手前（起動段階）で断つ方法。
- `run`/`diag` は既定で起動時に自動 release する（`--no-release` で抑止）。走行後に
  立たせたくなければ `--restore` を付けない。
- sport_mode を案A/Bで無効化した場合、ランナー起動時の auto-release は
  「解除すべきモードが無い」状態になるだけで問題ない（`checkmode` も "no mode active" を許容）。

## `unitree` アカウントでの実施（標準運用ユーザ）

標準では `unitree` アカウントで操作する。調査結果（2026-06-16 実測）:

- `unitree`: `uid=999`, groups に **`27(sudo)`** を含み、`sudo -l` は **`(ALL : ALL) ALL`**。
  → フルsudo権限あり（NOPASSWD指定は無いのでパスワード入力が必要。標準は `123` のことが多い）。
- **`mscli` は sudo 無しで `unitree` のまま動く**（読み取り `getservice sport_mode` が
  `unitree` で成功）。master_service へIPCで指示するクライアントで、特権処理は root の
  master_service 側が行うため、サービス制御は非rootでも通る。
  - 注意: `unitree` の PATH に `/unitree/sbin` は無い → **フルパス** `/unitree/sbin/mscli` で呼ぶ。
- サービス定義JSON・`Legged_sport` は root 所有で `unitree` に書込権なし
  → ファイル編集系（案A/B）は **`sudo` 前提**。

### unitree での手順まとめ

| 方法 | 永続 | sudo | コマンド |
|---|---|---|---|
| 案C（一時停止） | × | 不要 | `/unitree/sbin/mscli stopservice sport_mode` |
| 案A（Start無効化） | ○ | 要 | JSON編集後 `sudo /unitree/sbin/mscli reloadservice sport_mode` |
| 案B（バイナリmv） | ○ | 要 | `sudo mv .../Legged_sport{,.disabled}` |

```bash
# 一時停止（再起動で復活・sudo不要）
/unitree/sbin/mscli stopservice sport_mode

# 永続・案A（unitree + sudo）
sudo cp /unitree/etc/master_service/service/sport_mode ~/sport_mode.json.bak
sudo sed -i 's#--exec /unitree/module/sport_mode/Legged_sport#/bin/true#' \
    /unitree/etc/master_service/service/sport_mode   # ※Startのみ書換わるか要確認
sudo /unitree/sbin/mscli reloadservice sport_mode
/unitree/sbin/mscli stopservice sport_mode           # 伏せ状態で

# 永続・案B（unitree + sudo）
sudo mv /unitree/module/sport_mode/Legged_sport /unitree/module/sport_mode/Legged_sport.disabled
/unitree/sbin/mscli stopservice sport_mode
```

> 上記 sed は Stop/Status 行の `--exec` も置換してしまうため、実際にはエディタで
> `Start` の `Cmd` だけを `"/bin/true"` に差し替えるのが安全。

## mscli で enable を直接 disable できるか → できない（確定）

`/unitree/sbin/mscli` 内の `master_service.proto`（protobuf）を解析した結果:

- **`unitree.ms.ServiceState`** はフィールド `name` / `status` / `starttime` / **`enable`** を持つが、
  enable は**読み取り専用**（`getenable` / `listservice` で参照するのみ）。enable を書く RPC は無い。
- **`unitree.ms.SaveServiceParameter`** のフィールドは **`name` と `config` のみ**。
  → `saveservice [name][config]` は **Start/Stop/Status の実行コマンド(JSON)を保存するだけ**で、
    enable フラグは設定できない。
- コマンド一覧に `setenable` / `disableservice` / `enableservice` の類は**存在しない**。

結論: **`enable:0`（自動起動フラグ自体）は暗号化 `plan`/`init`/`manual` 側にあり、mscli からは変更不可。**
`advanced_sport` 等が `enable:0` なのは出荷時 plan の記述によるもので、mscli で後から落とす口は無い。

### ただし mscli だけで「永続的に立たせない」は可能

- **案A を mscli で（推奨・sudo不要の見込み）**: `saveservice` で sport_mode の Start を
  `/bin/true` に上書き保存。enable は 1 のままだが起動時に何も実行されず立たない。mscli は
  `unitree` のまま動くので **sudo もファイル編集も不要**でできる見込み。復旧は元 JSON で
  `saveservice` し直すだけ。
  ```bash
  # config は Start.Cmd を "/bin/true" にした JSON 文字列を渡す
  /unitree/sbin/mscli saveservice sport_mode '{"Start":{"Cmd":"/bin/true","ExpectCode":[0]},"Stop":{...},"Status":{...}}'
  /unitree/sbin/mscli reloadservice sport_mode
  /unitree/sbin/mscli stopservice  sport_mode   # 伏せ状態で
  ```
- **`removeservice sport_mode`**: サービス定義ごと削除。自動起動しなくなるが**破壊的**
  （復旧は元 config で `saveservice` 必須）。暗号化 plan まで消えるかは未検証。非推奨。

## 未確定・今後の検討

- 暗号化 `plan` 内の enable フラグを直接 `0` にする正規手段は未特定
  （`mscli` に `setenable` 無し。`saveservice`/`removeservice` が plan を書き換えるかは未検証＝
  ブート設定破損リスクがあるため未実施）。当面は案A/Bが安全・確実。
- `bashrunner`（enable:1, `/unitree/module/bashrunner/bashrunner.py`）は起動時に
  スクリプトを走らせるフックとして使える可能性があるが、立ち上がり前に止める
  タイミング保証が難しいため非推奨。
