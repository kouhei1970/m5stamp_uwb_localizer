# Wi-Fi ダッシュボードと無線コンソール（`uwb_net`）

タグ・アンカーの両ファームに組み込まれた `components/uwb_net`（Wi-Fi 接続 + ブラウザ用
ダッシュボード + 無線コンソール）の使い方をまとめます。USB を挿さずに、PC のブラウザで
測距・測位の状態を見たり、Wi-Fi 経由でコンソールコマンドを打ったりできるようにする機能です。

> 略語は本書の初出箇所でそのつど説明します。より広い用語は
> [`docs/GLOSSARY.md`](GLOSSARY.md) を参照してください。

**背景**: これまでは USB でホスト PC につないでログを採るしかなく、機材同士の距離が
USB ケーブルの長さに制限されていました。`uwb_net` を使うと、**書き込みは有線のまま**、
測距が始まった後は**モバイルバッテリで給電し、無線だけで監視・操作**できます
（設計の経緯は [`docs/HANDOFF.md` §0-E](HANDOFF.md)）。

---

## 1. 接続構成（トポロジ）

2 通りの構成をコンソールの `wifi mode` で選べます。どちらも UWB 測距そのものには
影響しません（Wi-Fi は 2.4GHz 帯、UWB は ch9〈約 8GHz〉なので周波数が重ならないため）。

### 1.1 A: 既存のルーター経由（STA）

全機を家庭・現場の Wi-Fi ルーターに **STA**（Station、子機モード。既存のルーターに
接続する動作）で参加させます。ルーターがある場所ならこちらが簡単です。

```
              Wi-Fi ルーター
             ╱      |       ╲
       STA接続   STA接続    STA接続
          │         │          │
        タグ      アンカー   PC のブラウザ
      "uwb-tag"   "uwb-anchor-0002"...  → http://uwb-tag.local/
```

### 1.2 B: タグを親機にする（SoftAP）

ルーターが無い現場では、**タグ自身を SoftAP**（Software Access Point、親機モード。
機器自身が Wi-Fi アクセスポイントになる動作）にして、アンカーと PC がタグへ直接つなぎます。

```
            タグ = SoftAP "uwb-tag"
           ╱                      ╲
      STA接続                  Wi-Fi接続
         │                         │
     アンカー                 PC のブラウザ
   "uwb-anchor-0002"...    → http://192.168.4.1/ （SoftAP の既定 IP）
```

### 1.3 自動モード（`auto`）は同時リセットからも自動復旧する

**既定の `mode=auto` は「NVS に SSID があれば STA で 20 秒繋ぎに行き、繋がらなければ
SoftAP へ切り替える」という動きです**（詳細は §3「モードの挙動」）。当初はここに
落とし穴がありました: 構成 B でタグ・アンカーを同時にリセットすると、タグ自身も
`auto` のため最初の 20 秒はルーターへの接続を試みてから SoftAP を立てる一方、
アンカーも同じ 20 秒でタイムアウトして自分の SoftAP へ切り替わってしまい、
**両者が別々の SoftAP に分かれて二度と合流できない**という問題でした。

**この問題は解決済みです。** `auto` モードで一度 SoftAP へフォールバックした後、
デバイスは **30 秒ごとに、保存済みの SSID だけを狙った能動スキャン**（SoftAP は
張ったままバックグラウンドで一時的に APSTA へ切り替えて行う。クライアントの
接続は切れない）を行い、見つかったら自動的に STA へ戻ります。つまり、タグが
少し遅れて SoftAP を立てても、アンカーは 30 秒周期のスキャンでいずれ見つけて
合流します。

**実機での確認（n04、2026-08-31）**: タグとアンカーを同時にリセットしたところ、
アンカーは 21 秒でいったん自分の SoftAP へフォールバックし、**55 秒後に
`uwb-tag` を発見して自動的に STA として合流した**（IP `192.168.4.2`）。
**手動操作は一切不要だった。**

**前提条件は変わりません**: この自動復旧が働くのは、アンカー側が事前に
`wifi set uwb-tag uwb-localizer` 等でタグの SSID を NVS に覚えている場合だけです。
SSID を一度も設定していないアンカーは、`auto` では最初から自分の SoftAP を
立てるだけで（20 秒待ちすら発生しない）、探しに行く先が無いので合流しません。

`wifi mode sta`（常時 STA・無期限リトライ・SoftAP へのフォールバック自体が無い）も
引き続き使えますが、**構成 B を成立させるために必須ではなくなりました**。
確実に STA だけで待ちたい場合の選択肢として残っています。

```
uwb-anchor> wifi set uwb-tag uwb-localizer   # 一度だけ。以後は再起動しても自動で覚えている
```

> 手動の `wifi scan`（§3）とこの自動再スキャンは同じ内部フラグを取り合って
> **同時には走りません**（片方が実行中なら、もう片方は次の機会に回ります）。

---

<a id="quickstart"></a>

## 2. クイックスタート

### 2.1 ビルド前に: 既存の `sdkconfig` を消す

`uwb_net` の追加でフラッシュサイズとパーティション構成が変わりました
（8 MiB フラッシュ・factory パーティション 3 MiB。§11）。**このリポジトリを
新しく `pull` した直後は、[`docs/GETTING_STARTED.md` §6.2](GETTING_STARTED.md#anchors5-console)
と同じ理由で `firmware/tag/sdkconfig` と `firmware/anchor/sdkconfig` を必ず作り直してください。**
古い `sdkconfig` が残っていると、フラッシュサイズ・パーティションの新しい既定値が
反映されず、ビルドや書き込みが失敗するか、古い 1 MiB パーティションのままになります。

```sh
rm -f firmware/tag/sdkconfig firmware/anchor/sdkconfig
```

### 2.2 ビルドと書き込み

```sh
. ~/esp/esp-idf/export.sh

cd firmware/tag
idf.py set-target esp32s3      # 初回のみ
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

アンカーも同様です（`cd firmware/anchor` で同じ手順）。**初回ビルドは
`espressif/mdns`（マネージド・コンポーネント。ESP-IDF 本体に含まれない外部
コンポーネントを Component Registry から取得する仕組み）を自動取得するため
インターネット接続が要ります。** 取得後は `managed_components/` に
キャッシュされ、以後はオフラインでビルドできます（mDNS 自体が要らなければ
`CONFIG_UWB_NET_MDNS=n` でこの取得自体を省略できます。§11）。

### 2.3 Wi-Fi を設定する

`monitor` で接続したコンソールから:

```
uwb-tag> wifi set lab-wifi-example ＜パスワード＞
ssid=lab-wifi-example を保存し、STA で接続を試みています
```

起動ログ（または再起動後のログ）に次の 1 行が出ます。**この行に IP アドレスと
URL が両方載っています**:

```
I (xxx) uwb_net_wifi: net: mode=sta ssid=lab-wifi-example ip=192.168.2.50 url=http://192.168.2.50/ mdns=http://uwb-tag.local/
```

### 2.4 ブラウザで開く

PC を同じ Wi-Fi（ルーターまたはタグの SoftAP）に繋いだ状態で、上のログの URL を
そのまま開くか、**mDNS**（Multicast DNS。IP アドレスを知らなくても `uwb-tag.local`
のような名前で機器を見つける仕組み）が使える環境（macOS は標準対応、Windows は
Bonjour 等が必要）なら:

```
http://uwb-tag.local/
```

アンカー単体のページは `http://uwb-anchor-0002.local/`（またはそのアンカーの IP）
で開けます。**1 画面で全機を見るにはタグのページを開いてください**（§8.2 の集約の説明）。
PC が今すぐ Wi-Fi を持って行けない場合の代替確認方法は §6（`net probe`）を参照してください。

---

## 3. `wifi` コンソールコマンド

USB シリアルコンソール・TCP コンソール（§5.1）・ブラウザのコンソール欄（§5.2）の
**どこから打っても同じ**です（`esp_console_run()` を共有しているため）。

| コマンド | 動作 |
|---|---|
| `wifi` | 状態を表示（モード・SSID・接続状態・IP・RSSI／AP クライアント数・WebSocket クライアント数・TCP コンソール接続有無・UDP 送受信数・行の破棄数） |
| `wifi set <ssid> [password]` | SSID/パスワードを **NVS**（Non-Volatile Storage、ESP-IDF の不揮発設定領域。電源を切っても値が残る）へ保存し、その場で STA として接続を試みる。パスワード省略はオープンネットワーク。8 文字未満のパスワードは拒否される（ESP-IDF の WPA2 実装の制約）。**制御文字（改行等）を含む SSID/パスワードも拒否される** |
| `wifi mode auto\|sta\|ap` | 起動モードを NVS へ保存する。`sta`（SSID 設定済みの場合）と `ap` はその場でも切り替わる。`auto` は次回起動から反映 |
| `wifi scan` | 周辺の Wi-Fi アクセスポイントをスキャンし、SSID・チャネル・RSSI・認証方式を一覧表示する（`AP not found` と「パスワード間違い」の切り分け用。SoftAP 動作中でも一時的に STA を起こしてスキャンする）。**SoftAP 動作中に打つと、スキャンの数秒間（目安 4 秒）だけ APSTA へ切り替わって他チャネルも走査する。接続中のクライアントは切れないが、その数秒はごく短い通信の途切れが起きうる** |
| `wifi clear` | SSID/パスワード/モードを消去し、次回起動から `mode=auto`・SSID 未設定に戻す |

> **`wifi set`/`wifi mode`/`wifi scan` は、スキャンが進行中（手動 `wifi scan` 実行中、
> または §1.3 の自動再スキャン中）だと `scan in progress, try again` とだけ返して
> 何もしません。**数秒待って打ち直してください。

### 状態表示の例

```
uwb-tag> wifi
mode         : sta (NVS 設定: auto)
ssid         : lab-wifi-example
password     : (set)
state        : connected
ip           : 192.168.2.50
rssi         : -45 dBm
ws clients   : 1
tcp console  : idle
udp tx/rx    : 0 / 5808
line drops   : 0
```

**パスワードそのものは絶対に表示されません**（`(set)` / `(none)` のみ）。

### モードの挙動（`uwb_net_wifi.cpp`、実装どおり）

| モード | 挙動 |
|---|---|
| `auto`（既定） | NVS に SSID があれば STA で接続を試みる。**20 秒**以内に IP が付かない、または SSID 未設定なら SoftAP へ切り替える。SoftAP へ切り替わった後も **30 秒ごとに保存済み SSID を探しに行き、見つかれば STA へ自動復帰する**（§1.3） |
| `sta` | 常に STA。切断されたら **2 秒**間隔で無期限に再接続を試みる |
| `ap` | 常に SoftAP。フォールバックの待ちが無く、即座に立ち上がる |

SoftAP の SSID は機体名そのもの（`uwb-tag` / `uwb-anchor-0002` 等）、パスワードは
Kconfig `UWB_NET_AP_PASSWORD`（既定 `uwb-localizer`）、チャネルは `UWB_NET_AP_CHANNEL`
（既定 1）、IP は ESP-IDF の既定 `192.168.4.1`、同時接続数上限は 4 台です。

### STA 接続時の起動ログの読み方

`lab-wifi-example` のような SSID を持つルーターへ STA として繋ぎに行くとき、
`uwb_net_wifi` タグで次のようなログが順に出ます（すべて USB・TCP コンソール
共通で、`ESP_LOG` 経由なので USB シリアルにのみ出ます。ネットワークへは流れません）。

```
I (xxx) uwb_net_wifi: net: STA connecting to ssid=lab-wifi-example
I (xxx) uwb_net_wifi: net: associated, waiting for DHCP
W (xxx) uwb_net_wifi: net: STA disconnected ssid=lab-wifi-example reason=201 (NO_AP_FOUND)
```

1 行目は接続開始（`WIFI_EVENT_STA_START`）、2 行目は 802.11 の関連付け
（associate）が済んで DHCP 待ちに入った合図（`WIFI_EVENT_STA_CONNECTED`）、
3 行目は切断（`WIFI_EVENT_STA_DISCONNECTED`。切断は本来何度も起こりうるため、
最大 2 秒に 1 行へ間引いて出力）です。**2 行目が出ずに 3 行目だけ出る場合は
関連付け自体に失敗しており、`reason` の値で原因を切り分けられます**（値は
ESP-IDF の `wifi_err_reason_t`。実機での誤解を避けるため、コード内コメントで
実際の定義値を確認済み）:

| `reason` | 名前 | 主な原因 |
|---:|---|---|
| **201** | `NO_AP_FOUND` | **指定した SSID の電波が見えていない。** SSID の打ち間違い・その SSID が 5GHz 専用（ESP32-S3 は 2.4GHz のみ対応）・単に電波が届かない、のいずれか。**パスワードの問題ではない** |
| **202** | `AUTH_FAIL` | 主に**パスワード間違い** |
| **15** | `4WAY_HANDSHAKE_TIMEOUT` | WPA2 の鍵交換（4-way handshake）がタイムアウト。多くは**パスワード間違い** |
| **204** | `HANDSHAKE_TIMEOUT` | 同上（ESP-IDF 側が合成する上位の分類） |

**切り分けの実務**: `reason=201` が出たら、まず `wifi scan`（§3 表）でその
SSID が実際に見えているか確認してください（2.4GHz の電波として出ていない
SSID は接続を何度リトライしても繋がりません）。`reason=202`/`15`/`204`系が
出るなら SSID 自体は見えているので、パスワードを確認して `wifi set` を
打ち直してください。「2 行目（associated）までは出るが GOT_IP のログ
（§2.3）が出ない」場合は認証ではなく DHCP（ルーター側のアドレス払い出し）
の問題です。

---

## 4. ブラウザダッシュボードの見方

`http://<ip>/` を開くと `components/uwb_net/web/index.html`（ビルド時にファーム
バイナリへ埋め込まれる 1 枚の HTML。外部 CDN には依存しません）が表示されます。

| パネル | 内容 |
|---|---|
| **ヘッダ** | 機体名・役割（タグ/アンカー）・Wi-Fi モード/SSID/IP/RSSI・稼働時間・WebSocket 接続状態（接続中／再接続中）・受信レート（行/秒・KB/秒） |
| **ノード** | 自機と（タグなら）集約された全アンカーのカード。空きヒープ・PHY 設定・PLL 粗調整コード・方式（SS/DS）・破棄数・WebSocket クライアント数・UDP 送受信数。アンカーのカードはさらに Poll/Response/Final/Result 数・再起動数・RX エラー/TX 失敗数・距離の平均±標準偏差も出す。**5 秒間新しい行が来ないカードは薄く表示される**（`stale`） |
| **アンカー一覧** | タグの `stats` 行から作る表（試行数・成功数・成功率・再試行数・救済数・初回成功率・現在の距離・所要時間・有効/無効・座標） |
| **チャート（直近 60 秒）** | 距離 `meas.d`、成功率 `stats`（太線=瞬時、細線=累積）、`fix.cycle_ms`（薄い帯=直近 1 秒の最大値） |
| **平面図** | 登録済みアンカーの座標、現在の測距値を半径とする円、位置解の軌跡（最大 200 点）、最新の推定位置 |
| **コンソール** | §5 参照 |
| **生ログ**（折りたたみ） | 直近 100 行の生の JSON をそのまま表示。`type` で絞り込み可能 |

### 記録・保存

ヘッダ右上の **「● 記録開始」**を押すと、以後受信した行をブラウザのメモリに
溜め始めます（ブラウザ内で完結し、デバイス側にはファイルを持ちません）。
**「保存」**を押すと `uwb_<機体名>_<日時>.jsonl`（1 行 1 JSON の
[JSON Lines](https://jsonlines.org/) 形式）としてブラウザのダウンロード先へ保存されます。
記録を続けたままページを閉じると記録内容は失われるので、区切りの良いところで保存してください。

> **記録される行はネットワーク経路の間引き後（§8.4）です。** `meas`/`fix` は
> 最大 20Hz に間引かれるため、**高頻度の生データ（〜100Hz、1 アンカー時）が
> 必要な解析には USB シリアルの記録
> （[`docs/GETTING_STARTED.md` §8.4](GETTING_STARTED.md)）を使ってください。**
> ブラウザ記録は「無線で概況を見る・軽い記録を残す」用途、USB 記録が
> フルレートの一次資料という位置づけです。

---

## 5. 無線コンソール

USB を挿さずにコンソールコマンドを打つ手段が 2 つあります。どちらも
**同じ `esp_console_run()`** を通すので、使えるコマンドは USB コンソールと
完全に同じです（`anchor list` / `wifi` / `output` / `net probe`（§6） / `info` /
`help` など）。

### 5.1 TCP コンソール（`nc` / `telnet`）

```sh
nc uwb-tag.local 23
# または
telnet 192.168.2.50 23
```

```
uwb-net console (uwb-tag). Type 'exit' to close.
uwb-tag> anchor list
idx  addr       x[m]      y[m]      z[m]   delay[m]  enabled
  0  0x0002     0.000     0.000     2.400     0.0000  yes
uwb-tag> exit
bye
```

平文 TCP（暗号化なし）、**同時に 1 クライアントまで**（2 台目は前のセッションが
終わるまで接続待ちになります）。1 行の最大長は 160 文字。telnet のネゴシエーション
（IAC シーケンス）は読み捨てます。

**注意: この TCP コンソールは対話的なコマンド応答専用で、`meas`/`fix` などの
JSON 行を自動では流しません。** `nc` で繋いだままにしても測距データは見えません
（見たいならブラウザの WebSocket、または §8.2 の UDP、[`docs/GETTING_STARTED.md` §8.4](GETTING_STARTED.md) の USB シリアル記録を使ってください）。

### 5.2 ページ内の端末欄

ダッシュボードの「コンソール」パネルに直接コマンドを打てます。入力欄で
`↑`/`↓` キーが履歴呼び出しです。クイックボタン（`info` / `anchor list` /
`wifi` / `help`）も用意してあります。内部では WebSocket で `{"cmd":"..."}`
を送り、実行結果が `"type":"con"` の行として返ってきます（§8.1）。

**コマンドの実行は専用のワーカータスク `uwb_net_wscmd`（コア0・優先度4・
スタック8192）が担います。** HTTP/WebSocket サーバ本体のタスク（`httpd`）は
受け取ったコマンドをこのワーカーへ渡すだけで、自分では実行しません。
これにより、`wifi scan`（数秒）のような時間のかかるコマンドを実行している間も、
チャート等へのストリーミング配信は止まりません（§6 で `net probe` により実機確認済み）。
**ワーカーは長さ1のキューで動くため、同時に処理できるコマンドは1個だけです。**
実行中にもう1つコマンドが来ると、待たずにその場で
`{"ret":-1,"text":"busy: another command is running"}` という `con` 行が
即座に返ります（キューに並んで後で実行されるわけではないので、少し待って
打ち直してください）。

### 5.3 実装メモ: 標準出力の差し替え

ESP-IDF の newlib は FreeRTOS タスクごとに別々の内部状態を持つため、
リモートコマンドの実行中に行う `stdout`/`stderr` の一時的な差し替えは
**コマンドを実行したタスク（TCP セッションのタスク、または上記の
`uwb_net_wscmd` ワーカー）自身にしか効かず**、USB シリアルの REPL や
他のリモートコマンドの出力とは混ざりません（同時実行は内部のミューテックスで
1 個ずつに直列化されます）。

---

## 6. ネットワーク自己診断（`net probe`）

**PC のブラウザが無くても、もう 1 台のデバイスの USB コンソールから
ネットワーク経路一式を検証できるコマンドです。** 現場でブラウザが用意できない
段階でも、HTTP・TCP コンソール・WebSocket が正しく動いているかを確認できます
（§7 の「ブラウザからの確認」を補う手段。ページの描画そのものは確認できません）。

```
net probe <host> [http_port] [console_port] [ws_cmd]
# 既定 http_port=80, console_port=23, ws_cmd="wifi"
```

もう 1 台のデバイス（例: アンカー）の USB コンソールから、確認したい相手
（例: タグの IP）を狙って打ちます。実行すると、そのデバイス自身がクライアントに
なって相手の HTTP/TCP/WebSocket を素のソケットで叩き、結果を（打った側の）
USB コンソールへ表示します。1 ステップが失敗しても後続のステップは続けます。

| 手順 | 確認内容 |
|---|---|
| 1. HTTP | `GET /api/info` と `GET /` のステータス行。`GET /` はレスポンスヘッダの `Content-Length` と実際に受信できたバイト数を突き合わせる |
| 2. TCP コンソール | 接続直後のバナー文字列と、第4引数のコマンド（既定 `wifi`。WebSocket 側と共通）を送って返ってきた応答（先頭 400 バイト） |
| 3. WebSocket | ハンドシェイク（`101 Switching Protocols`）の確認、3 秒間に届いたフレーム数・バイト数・行の `type` 別件数の集計、第4引数 `ws_cmd`（既定 `wifi`）のコマンドを送って `"type":"con"` の応答が **10 秒以内**に返るかの確認 |

> `con` 応答の待ちは 10 秒です（§5.2 のワーカータスク化により `wifi scan` の
> ような数秒かかるコマンドでも待ちきれるようにした）。第4引数へ好きなコマンドを
> 渡せます。**このコマンドは TCP コンソール・WebSocket の両方へ同じものが送られる**
> ので、`net probe <host> 80 23 info` と打てば、`info` コマンドを USB（自分自身が
> 打った `net probe` 自体は USB コンソールから実行）・TCP コンソール・WebSocket の
> **3 経路すべて**で確認できます。例えば `net probe 192.168.4.1 80 23 "wifi scan"` なら、
> 相手機の `wifi scan`（§3、約4秒）を TCP・WebSocket の両方から遠隔実行し、その応答
> （AP 一覧）をそのまま受け取れます。
>
> **接続そのものにも実タイムアウトがあります**（3 秒。以前は接続を確立できない
> 相手だと TCP の OS 側リトライ〈数十秒〉まで固まっていた問題を、非ブロッキング
> `connect()` + `select()` で修正済み）。相手が電源断・IP 間違い等で応答しない
> 場合は `probe: connect timeout (host=... port=...)` とだけ表示してそのステップを
> 諦め、次のステップへ進みます。

### 実機での出力例（2026-08-31。アンカー〈192.168.4.2〉からタグ〈192.168.4.1〉へ）

```
probe http: GET /api/info -> HTTP/1.1 200 OK
probe http: GET / -> HTTP/1.1 200 OK
probe http:   Content-Length=31396 body_bytes_received=31396 (一致)
probe tcp: banner: uwb-net console (uwb-tag). Type 'exit' to close.
probe tcp: 応答[0:400]=
 uwb-tag> mode : ap (NVS 設定: auto) ... ws clients : 0 / tcp console : connected / udp tx/rx : 0 / 65
probe ws: HTTP/1.1 101 Switching Protocols
probe ws: frames=55 bytes=38607 types: meas=54 fix=54 anchor_stats=3 node=6 stats=3
probe ws: con応答[0:300]={"v":1,"type":"con","cmd":"wifi","ret":0,"text":"mode         : ap ..."}
```

**この出力の読み方**:

- `Content-Length=31396 body_bytes_received=31396 (一致)` — ダッシュボードの
  HTML 本体が壊れず最後まで届いていることの確認（サイズが一致しない場合は
  途中で切れている、または `Content-Length` の計算がおかしいことを疑う）。
- `meas=54 fix=54`（3 秒間）— 1 秒あたり約 18 行で、§8.4 の 50ms 間引き
  （最大 20Hz）とほぼ一致します。
- `node=6`（3 秒間）— `node` 行は 1 秒に 1 回、**タグ自身の分とアンカー集約分の
  2 台ぶん**が流れるので、3 秒で約 6 件が期待どおりです。
- `anchor_stats=3` — アンカーの統計行が UDP 経由でタグに集約され、タグの
  WebSocket からブラウザ側にも流れていることの確認（§8.2 の集約の実地確認）。

### 重いコマンドでもストリームは止まらない（実機確認）

`net probe 192.168.4.1 80 23 "wifi scan"` を実行し、第4引数で相手機
（タグ）の `wifi scan`（§3。約4秒かかる）を WebSocket 経由で遠隔実行したところ、
**その約4秒の実行中もダッシュボードへのストリーム配信は止まらず**、
`con` 応答にはスキャン結果（AP 12 件のテーブル）がそのまま返ってきました。
これは §5.2 で述べたワーカータスク化（コマンド実行と配信を別タスクに分離）の
実地確認です。**以前の実装（コマンドを httpd タスク上で同期実行していた版）
なら、この間ダッシュボード全体が固まっていました。**

### `info` / `anchor list` の実機確認（コンソール飢餓バグ修正後）

[`docs/HANDOFF.md` §0-D 項目8](HANDOFF.md) の潜在バグ（アンカー登録テーブルの
排他ロックを取るコマンドが無期限に固まりうる件）を修正した後、`info` と
`anchor list`（どちらもこのロックを取るコマンド）が USB コンソール・TCP コンソール・
WebSocket の**3経路すべてで正常に応答することを確認しました**。

- `net probe <tag_ip> 80 23 info` をアンカーの USB コンソールから 3 回実行し、
  **3 回とも成功**（TCP・WebSocket 双方の `info` 応答が正常に返った）。
- 起動直後の1回目の試行でだけ、WebSocket のフレーム件数の表示が乱れる現象が
  出ましたが、**単発で再現しませんでした**（原因未特定。実害〈`con` 応答自体は
  正しく届いていた〉は無かったため、これ以上の追跡は保留）。

### この結果からの結論

**この 1 回の実行だけで、`uwb_net` の HTTP・TCP コンソール・WebSocket（配信・
コマンド往復の両方）・UDP 集約のすべてが、別デバイス視点で正しく動作している
ことを確認できました。** 未確認なのは「実際の PC のブラウザでページを開いた
ときの見た目・操作感」だけです（§7）。

---

## 7. ブラウザからの確認

### 確認できたこと（PC のブラウザなしで、2026-08-31）

- `net probe`（§6）により、**HTTP・TCP コンソール・WebSocket（配信・コマンド
  往復）・UDP 集約は、別デバイスから見て正しく動作していることを確認済み**
  （§6 の実機出力）。
- §1.3 の自動再スキャンによる復旧も**実機（n04）で確認済み**（手動操作なしで
  30〜55 秒程度で自動的に合流）。
- ダッシュボードページの JavaScript（チャート描画・アンカー一覧・コンソールの
  往復・記録/保存・狭い画面幅でのレイアウト）は、**実機を使う前に Chrome で、
  実際に採取したログを再生するモックサーバを相手に動作確認済み**です。

### まだ確認できていないこと

**実際の PC のブラウザで、生きているデバイスにネットワーク越しに接続して
ページを開くこと** はまだ行っていません。

**理由（2026-08-31 時点）**: 研究室の SSID（固有名は本書に記載しない）が、デバイスを
置いた場所からは電波として届いていませんでした（`wifi scan` で見えたのは
別の 2.4GHz ネットワークだけで、当該 SSID 自体が一覧に出なかった）。
一方 PC 自身は 5GHz 専用のネットワークに繋がっており、ESP32-S3 は 2.4GHz
専用なのでそのネットワークには参加できません。さらに、無人運転中に PC の
Wi-Fi をタグの SoftAP へ切り替えると PC 自身が他の作業からネットワークごと
切り離されてしまうため、その切り替えは意図的に行いませんでした。

### 次回、確認する方法（どちらか）

**(a) ルーター経由（構成 A、§1.1）**: タグ（と各アンカー）の USB コンソールから、
PC も繋がる 2.4GHz のルーターを指定します。

```
uwb-tag> wifi set <2.4GHzのSSID> <パスワード>
```

起動ログ（§2.3）に出る URL、または `http://uwb-tag.local/` を PC のブラウザで開く。

**(b) タグを親機にする（構成 B、§1.2）**: PC の Wi-Fi を SoftAP `uwb-tag`
（パスワードは Kconfig `UWB_NET_AP_PASSWORD`、既定 `uwb-localizer`）へ
接続し、`http://192.168.4.1/` を開く。

### 確認すべき項目

- [ ] `http://uwb-tag.local/` （または IP 直打ち）でダッシュボードが開くこと
- [ ] ヘッダ・ノードカード・アンカー一覧・チャート・平面図が実データで更新されること
- [ ] 「記録開始」→「保存」で `.jsonl` が実際にダウンロードされ、中身が読めること
- [ ] ページ内コンソールから `anchor list` 等を打って応答が返ること
- [ ] 複数ブラウザタブを同時に開いても壊れないこと

---

## 8. データプロトコル

### 8.1 行の種類（JSON Lines、末尾は必ず `\n`、全行 `"v":1`）

| 種別 | 出どころ | 頻度 | 内容 |
|---|---|---|---|
| `anchors` | タグ | 起動時・表が変わるたび | 登録済みアンカーの一覧（座標・アンテナ遅延・有効/無効） |
| `meas` | タグ | 〜100Hz（ネットワークへは§8.4で間引き） | 生の測距値 |
| `fix` | タグ | 同上 | 測位結果（`p`・`gdop`・`residual_rms`・各アンカーの内訳など） |
| `stats` | タグ | 1Hz | アンカーごとの累積成功率・再試行・救済数 |
| `anchor_stats` | アンカー | 1Hz | Poll/Response/Final/Result 等の累積カウンタ |
| `range` | アンカー | DS-TWR 交換ごと（`CONFIG_UWB_ANCHOR_LOG_EVENTS=y` のときのみ） | 個々の交換の距離・所要時間 |
| **`node`**（新規） | 全機 | 1Hz。**USB へも常に出力** | 機体名・役割・Wi-Fi 状態・空きヒープ・稼働時間・破棄数・WS/TCP/UDP のカウンタ + 役割固有の付加情報（PHY 設定等） |
| **`con`**（新規） | 全機（WebSocket 経由のコマンド応答） | コマンドが来たときだけ、**発行したクライアントにのみ** | コマンドの実行結果（JSON エスケープ済みテキスト） |

`node` 行の例（タグ）:

```json
{"v":1,"type":"node","t":123.456,"role":"tag","name":"uwb-tag","addr":"tag0",
 "wifi":{"mode":"sta","ssid":"lab-wifi-example","ip":"192.168.2.50","rssi":-45,"clients":0},
 "uptime_s":123,"heap_free":165000,"heap_min":150000,"drops":0,"ws_clients":1,
 "tcp_console":false,"udp_tx":0,"udp_rx":5808,
 "phy":"850k/pre256/pac8/ch9","pll_coarse":"0x23","method":"DS","retry_max":2,"retry_delay_ms":2}
```

### 8.2 転送経路

| 経路 | 内容 |
|---|---|
| `HTTP GET /` | ダッシュボード本体（`text/html`） |
| `HTTP GET /api/info` | 現在の `node` 行と同じ内容を JSON で返す（ポーリング用途） |
| `WebSocket /ws` | サーバ→ブラウザ: JSON Lines を**複数行まとめて 1 テキストフレームで**プッシュ配信。ブラウザ→サーバ: `{"cmd":"..."}` を送ると `con` 行が 1 個返る |
| **UDP ブロードキャスト**（`255.255.255.255:5006`、既定） | アンカー→タグ。アンカーは間引き後の行を 1400 バイト以内・行単位でブロードキャストし、タグは同じポートで受信して自分の配信へ混ぜる |
| **TCP コンソール**（既定ポート 23） | §5 参照。JSON Lines のストリーミングはしない |

**タグは集約役です**（`aggregate=true`）。タグの `/ws` を開いたブラウザには、
タグ自身の行に加えて**全アンカーの `anchor_stats`/`node` 行も UDP 経由で混ざって
届きます**（§6 の `net probe` 出力例の `anchor_stats=3` はこの経路の実地確認）。
アンカー単体のページは自分の行しか出しません。

### 8.3 WebSocket フレーミング

1 フレーム = 改行区切りの JSON 行が 1 個以上（最後の行も `\n` で終わる）。
ブラウザ側は `\n` で分割してから 1 行ずつ `JSON.parse` し、パースに失敗した行は無視します。

### 8.4 間引き（decimation）— `meas`/`fix`/`range` は最大 20Hz

`meas`・`fix`・`range` はネットワークへ配信する直前に間引かれます。既定の
`CONFIG_UWB_NET_HIGHRATE_MIN_INTERVAL_MS`（50ms）により、**同じ種別の行は
最短 50ms 間隔＝最大 20Hz でしかネットワークへ流れません**（アンカー 1 台なら
生の頻度は〜100Hz なので、5 行に 1 行程度になります）。**この間引きは
ネットワーク経路（WebSocket・UDP）だけに掛かります。USB シリアルへの JSON
Lines 出力は影響を受けず、常に生の頻度のままです。** `anchors`・`stats`・
`anchor_stats`・`node`・`con` は間引きの対象外（低頻度なのでそのまま流れます）。

---

## 9. Kconfig オプション

`idf.py menuconfig` → `Component config` → `UWB Network (Wi-Fi dashboard)`。

| オプション | 既定 | 内容 |
|---|---|---|
| `CONFIG_UWB_NET_ENABLE` | y | n にすると `uwb_net` の公開関数がすべて no-op になり、Wi-Fi・HTTP・UDP・TCP コンソールを一切起動しない（USB シリアルのみで運用したい場合や Wi-Fi の消費電力・干渉を避けたい場合） |
| `CONFIG_UWB_NET_AP_PASSWORD` | `uwb-localizer` | NVS に SSID が無いとき（買ったばかり／`wifi clear` 後）に立ち上げる SoftAP の WPA2 パスワード。SSID は機体名そのもの |
| `CONFIG_UWB_NET_AP_CHANNEL` | 1（1〜13） | SoftAP のチャネル。UWB（ch9）とは周波数帯が異なるので測距への直接の干渉は無い |
| `CONFIG_UWB_NET_HTTP_PORT` | 80（1〜65535） | ダッシュボード（`/`）と WebSocket（`/ws`）のポート |
| `CONFIG_UWB_NET_CONSOLE_PORT` | 23（1〜65535） | TCP コンソールのポート |
| `CONFIG_UWB_NET_UDP_PORT` | 5006（1〜65535） | アンカー→タグの UDP ブロードキャスト/受信ポート |
| `CONFIG_UWB_NET_HIGHRATE_MIN_INTERVAL_MS` | 50 | `meas`/`fix`/`range` 行をネットワークへ配信する最短間隔 [ms]。0 で間引きなし（§8.4） |
| `CONFIG_UWB_NET_RING_BYTES` | 12288 | 配信待ちの行を溜めるリングバッファ（.bss 静的確保）のサイズ [bytes]。溢れた行は捨てられ `node` 行の `drops` に計上される |
| `CONFIG_UWB_NET_MDNS` | y | n にすると mDNS を起動しない（IP アドレス直打ちのみでアクセス可能。ビルド時の `espressif/mdns` 取得〈§2.2〉も不要になる） |

**30 秒の自動再スキャン間隔（§1.3）や `net probe`（§6）の各タイムアウト（3 秒等）は
Kconfig 化されておらず、ソース中の定数です**（それぞれ `uwb_net_wifi.cpp` の
`kRescanIntervalUs`、`uwb_net_probe.cpp` 内の各タイムアウト）。

---

## 10. リソース使用量

### 10.1 バイナリサイズ（実測、2026-08-31 ビルド）

| ファーム | サイズ | factory パーティション（3 MiB）に対する使用率 |
|---|---:|---:|
| `firmware/tag`（`uwb_tag.bin`） | 1,068,848 バイト（約 1044 KiB） | 約 34%（残り約 2.0 MiB） |
| `firmware/anchor`（`uwb_anchor.bin`） | 1,010,672 バイト（約 987 KiB） | 約 32%（残り約 2.0 MiB） |

`uwb_net` 追加前は factory パーティションが 1 MiB だったため、Wi-Fi
スタック（`esp_wifi`）・`esp_http_server`・lwIP の TCP/UDP・mDNS を
組み込んだ分だけ 3 MiB へ拡張しています（§11 のフラッシュ構成）。

### 10.2 タスク構成（`uwb_net` が追加する分）

全て **コア 0**（UWB の電波を扱うタスクはコア 1 のまま、干渉させない設計）。

| タスク | コア | 優先度 | スタック | 役割 |
|---|:-:|:-:|---:|---|
| `uwb_net_tx` | 0 | 6 | 6144 | リングバッファを 50ms ごとに最大 3072 バイト排出し WebSocket/UDP へ渡す。1000ms ごとに `node` 行を組み立てる |
| `uwb_net_tcp` | 0 | 5 | 8192 | TCP コンソールの accept ループ + セッション処理（§5.1）。リモートコマンドの捕捉バッファもこのタスクのスタック上に載る |
| `uwb_net_udprx` | 0 | 5 | 4096 | UDP 受信（`aggregate=true` のタグのみ起動） |
| `uwb_net_wifi` | 0 | 4 | 4096 | §1.3 の自動再スキャンの実行タスク（`esp_timer` から通知を受けて実際のスキャンを行う。ブロックしうる処理を `esp_timer` タスク自身にやらせないための分離） |
| **`uwb_net_wscmd`**（新規） | 0 | 4 | 8192 | §5.2 のコマンド実行ワーカー。WebSocket 経由のコマンドを `httpd` タスクから受け取って実行し、結果を `httpd` タスクへ返す。長さ1のキューで直列化（同時1コマンドまで） |
| `httpd`（ESP-IDF 内部） | 0 | 5 | 10240 | HTTP/WebSocket サーバ本体。**コマンドの実行自体はもう行わない**（`uwb_net_wscmd` へ委譲。busy 応答の組み立てとフレーム送受信だけを担う） |
| Wi-Fi ドライバ内部タスク | 0 | — | — | `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0=y` で固定 |
| lwIP TCP/IP タスク | 0 | — | — | `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y` で固定 |

タグの `uwb_ranging_svc`（電波担当、コア 1・優先度 18）、アンカーの `uwb_radio`
（コア 1・優先度 20）は無変更です。`net probe`（§6）はコンソールコマンドとして
呼び出し元のコンソールタスク（USB REPL・TCP コンソール・HTTP サーバのいずれか）
上でそのまま動くため、専用タスクは持ちません。

### 10.3 スタック残量（実測、2026-08-31）

ファームは起動から約 10 秒後に、上表の `uwb_net` 側タスクのスタック残量
（`uxTaskGetStackHighWaterMark()`、そのタスクが今まで一番深く使った分を差し引いた
「最も少なかったときの空き」）を 1 回だけ USB シリアルへ自己ログします
（`"... task stack high-water mark: N bytes free"`）。タグでの実測値:

| タスク | スタックサイズ | 10 秒後の空き |
|---|---:|---:|
| `uwb_net_tx` | 6144 | 2724 バイト |
| `uwb_net_udprx` | 4096 | 3008 バイト |
| `uwb_net_tcp` | 8192 | 7164 バイト |
| `uwb_net_wifi` | 4096 | 3256 バイト |
| `uwb_net_wscmd` | 8192 | 7228 バイト |

いずれも 2KB 以上の余裕があります（[`docs/GETTING_STARTED.md` §8.6](GETTING_STARTED.md#ranging-service)
と同じ目安）。`uwb_net_tx` が最も余裕が薄い（6144 中 2724 空き＝使用量 3420
バイト）のは、`node` 行の組み立てバッファ（1024 バイト）と内部の `vsnprintf`
呼び出しをスタック上に積むためです（`uwb_net_sink.cpp` のコメント参照）。

### 10.4 ヒープ・NVS

- リングバッファ（`CONFIG_UWB_NET_RING_BYTES`、既定 12 KiB）は `.bss` に静的確保（ヒープを消費しない）。
- 実測の空きヒープは §12 参照（測距 + Wi-Fi + SoftAP + UDP 集約 + WebSocket 配信が同時に動いている状態で 150〜165 KB 程度）。
- NVS 使用は `uwb_net` の名前空間 `"uwb_net"` に `mode`/`ssid`（最大 32 文字）/`pass`（最大 64 文字）の 3 キーのみ。パーティション全体（24 KiB）に対して無視できるサイズです。

---

## 11. フラッシュ構成の変更

`uwb_net` の追加に伴い、両ファームの `sdkconfig.defaults`・`partitions.csv` を変更しました。

| 項目 | 変更前 | 変更後 | 理由 |
|---|---|---|---|
| フラッシュサイズ | （未指定＝既定 2/4 MiB 相当） | `CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y` | M5StampS3A は実測 8 MiB フラッシュ（`esptool flash_id` で確認済み） |
| factory パーティション | 1 MiB | **3 MiB**（`partitions.csv`、オフセットは変わらず `0x10000`） | Wi-Fi/HTTP/mDNS/lwIP を組み込んだ分のアプリサイズ増加を吸収 |
| `CONFIG_HTTPD_WS_SUPPORT` | n | **y** | ダッシュボードの WebSocket（`/ws`）に必須 |
| `CONFIG_HTTPD_MAX_REQ_HDR_LEN` | 既定値 | 1024 | 一部ブラウザの長いヘッダ（`Sec-WebSocket-Extensions` 等）でハンドシェイクが弾かれるのを防ぐ |
| `CONFIG_LWIP_MAX_SOCKETS` | 既定 10 | 16 | HTTP（最大6接続）+ UDP(1) + TCP コンソール(listen1+accept1) + mDNS の同時オープン分の余裕 |
| `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0` / `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0` | n | y | ネットワーク系タスクを一律コア 0 に固定し、コア 1（UWB 電波担当）へ持ち込まない |

**§2.1 で述べたとおり、この変更を取り込んだ後は古い `sdkconfig` を必ず削除してください。**

---

## 12. 実測結果（2026-08-31、実機）

条件: 卓上 1.2m、アンカー 1 台。すべて**タグを SoftAP（`uwb-tag`）、アンカーを
STA でタグへ接続**、UDP 集約を動かした状態（構成 B、§1.2）。n05 は計測中に
**アンカーがタグの `/ws` へ 3 回接続し直しながら**（WebSocket クライアントの
出入りが測距へ影響しないか見るため）、**n08（最終版）は 60 秒時点で TCP コンソール、
75 秒時点で WebSocket から、それぞれ `info`/`anchor list`（§0-D 項目8のロックを
取るコマンド）を実際に打ちながら**測距を継続した。

| 実行 | 秒数 | 周期成功率 | 周期時間（中央値/p95/最大） | アンカーのカウンタ | `tx_failures` | 行の破棄 | タグの空きヒープ |
|---|---:|---:|---|---|---:|---:|---|
| **n02** | 95 | **99.4%**（5286 周期） | 8ms / 18ms / 31ms | `polls=5808 responses=5808 finals=5512 results=5512` | 0 | 0 | 約 162 KB |
| **n05**（WS 接続 3 回） | 95 | **99.3%**（5307 周期） | 8ms / 18ms / 33ms | `polls=5745 responses=5743 finals=5476 results=5476` | 2（0.03%） | 0 | 156 KB（最小 134 KB） |
| **n07**（飢餓対策の暫定版＝1tick 版） | — | **99.3%**（5554 周期） | 8ms / 18ms / 33ms | （未記録） | 0 | （未記録） | （未記録） |
| **n08**（最終版。TCP/WS から `info`/`anchor list` を実行しながら） | 95 | **99.4%**（5519 周期） | **9ms** / 18ms / 31ms | `polls=5993 responses=5993 finals=5700 results=5700` | 0 | 0 | 140 KB（最小 126 KB） |

**n05 の `tx_failures=2`（0.03%）はその後再現していません**（n07・n08 とも 0）。
1 回限りの事象だった可能性が高いですが、原因は依然未特定のため結論は保留のままです。

**n07 は §0-D 項目8の飢餓対策の暫定版（1 tick だけ譲る版）で測ったものです。**
実機で core 0 に固定された TCP コンソール・WebSocket ワーカーが依然として飢えることが
分かり、**「待ち手がロックを取るまで、上限 20ms の範囲で 1 tick ずつ譲り続ける」最終版へ
差し替えました**（詳細は `docs/HANDOFF.md` §0-D 項目8）。**n08 はその最終版での計測です。**

**n08 では周期時間の中央値が 8ms → 9ms へわずかに伸びています。** これは§13 で述べる
とおり、リモートコマンド（今回は 60 秒・75 秒時点の `info`/`anchor list`）がロックを
握っている間、測距タスクがその1周期ぶんだけ足止めされるためで（1 回あたり最大 20ms、
通常 1〜3ms）、**測定全体でならすと中央値に 1ms 乗る程度の影響**です。周期成功率・
`tx_failures`・行の破棄には悪化が見られません。

**比較対象（Wi-Fi 無効の基準測定 e67、`docs/HANDOFF.md` §0-D「G-2」）**: 99.4%、
周期時間 8ms/16ms。**n02・n05・n07・n08 のいずれも e67 とほぼ同一**であり、
**Wi-Fi（2.4GHz）+ WebSocket ストリーミング + リモートコマンド実行 + コア 0 の
ネットワーク処理は、コア 1 の UWB 測距の周期成功率・周期時間を悪化させていません。**

行の破棄（ネットワーク側リングバッファ溢れ）は記録のある全ての実行で 0 件であり、
Wi-Fi 送信によって UWB の遅延送信（delayed TX）ウィンドウを取りこぼす様子も、
ネットワーク側のリングバッファが溢れる様子も、これらの条件では観測されていません。

---

## 13. 制約・注意点

- **Wi-Fi パスワードは NVS に平文で保存されます**（暗号化 NVS は未使用）。
  `wifi` コマンドの状態表示では絶対に表示されませんが、**実際の SSID/パスワードを
  設定ファイル・メモ・コミット等、リポジトリを含むいかなる場所にも書き残さないでください。**
- **`output off`（タグのみ）は USB だけでなく WebSocket/UDP への配信も止めます。**
  タグの `anchors`/`meas`/`fix`/`stats` の各出力関数（`firmware/tag/main/main.cpp`）は
  `jsonOutputEnabled()` が `false` のとき、USB への `fputs` と `uwb::net::publishLine()`
  の**両方を呼ばずに早期リターン**します。したがって `output off` にすると、USB
  だけでなくダッシュボードのチャート・アンカー一覧・平面図も更新が止まります。
  **`node` 行だけは例外**で、`uwb_net_tx` タスクが 1 秒ごとに独立して組み立てて
  USB と配信の両方へ出す（`uwb_net_sink.cpp`）ため、`output off` 中でも
  ヘッダのカード（機体情報・空きヒープ・Wi-Fi 状態）は動き続けます。
  **アンカー側には `output` コマンドが無く、常に出力されます**（この制約はタグのみ）。
  **タグを集約役にしている場合、タグの `output off` はタグ自身の行だけを止め、
  UDP 経由で混ざってくるアンカーの `anchor_stats`/`node` 行は止まりません。**
- **SoftAP パスワードは `wifi set` では変更できません。** `wifi set` は STA
  （自機が接続しに行く先）の認証情報だけを扱うコマンドです。SoftAP 側の
  パスワードを変えたい場合は `CONFIG_UWB_NET_AP_PASSWORD`（既定 `uwb-localizer`）
  を `menuconfig` で変更して再ビルドしてください。
- **省電力（Wi-Fi power save）は無効化されています**（`esp_wifi_set_ps(WIFI_PS_NONE)`）。
  レイテンシと UDP ブロードキャストの受信確実性を優先した設計で、代わりに
  **バッテリ駆動時の消費電流は Wi-Fi 省電力ありの構成より高くなります。**
- **モバイルバッテリの自動停止に注意してください**（`docs/HANDOFF.md` §0-E）。
  負荷が小さいと自動的に給電を止めるタイプのモバイルバッテリがあります。
  アンカーは受信を常時 ON にしているため常時それなりの電流を引きますが、
  実際に使うバッテリでの自動停止の有無は個体ごとに確認してください。
- **`espressif/mdns` はマネージド・コンポーネントです**（§2.2）。初回ビルド時に
  インターネット接続が必要です。取得後は `managed_components/` にキャッシュされ、
  以後はオフラインでビルドできます。mDNS 自体が不要なら `CONFIG_UWB_NET_MDNS=n`
  でこの取得自体を回避できます。
- **リモートコマンドの標準出力は呼び出しタスクだけに差し替わります**（§5.3）。
  USB シリアルの REPL と無線コンソール（TCP・WebSocket）が同時に動いても
  互いの出力が混ざることはありません。
- **`net probe`（§6）はローカルネットワーク内の既知 IP 専用です**（数値 IPv4
  アドレス限定。DNS 解決はしません）。ホスト名（`uwb-tag.local` 等）を渡しても
  動きません。IP アドレスは相手機の起動ログ（§2.3）や `wifi`（§3）で確認してください。
- **タグ側で見つかった潜在バグ（`uwb_net` 自体の不具合ではない。修正済み・実機確認済み）**:
  アンカー登録テーブルの排他ロックを取るコンソールコマンド（`info`・`anchor list` 等）が、
  全アンカーが応答し続けている状況で無期限にブロックしうる不具合が見つかり、修正して
  実機で解消を確認しました（原因・修正の詳細は
  [`docs/HANDOFF.md` §0-D 項目8](HANDOFF.md)。`components/uwb_ranging` 側の問題で、
  `uwb_net` 自体には手を入れていません）。§6 で `net probe` により、`info`/`anchor list`
  が USB・TCP コンソール・WebSocket の3経路すべてで正常応答することを確認済みです。
  **副作用として、こうしたコマンドを実行している間は測距が短時間だけ足止めされます**
  （1 回のロック取得あたり最大 20ms、実測では通常 1〜3ms 程度。§12 の n08 では
  この影響で周期時間の中央値が 8ms → 9ms に伸びましたが、周期成功率・`tx_failures`・
  行の破棄には悪化が見られませんでした）。
