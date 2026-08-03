# botrail 設計方針

> ロボットセルをコードで組み、決定的に検証し、USD で渡す — ROS 非依存・UI ファーストのセル・オーサリングライブラリ

## 1. ビジョン

**「`pip install botrail` と数行のコードだけで、ブラウザ上の 3D UI でロボットセルを組み立て、1 サイクルが成立することを決定的に検証し、USD で渡せる」**

### 1.1 成果物の単位は「1 本の軌道」から「1 サイクルのセル」へ

初期の botrail の成果物は 1 本の軌道だった(`plan_to_pose()` → CSV / URScript)。MoveIt との比較が成立するのはこの粒度である。しかし USD import(P3/P4)とシーケンス制御(S1〜S6)を経て、実際に出力されるものは一段上の粒度になった。

| | 初期 | 現在 |
|---|---|---|
| 入力 | URDF + 手で置いた障害物 | URDF/Xacro/USD ロボット + USD ステージ(セルのレイアウト・機器・設置フレーム) |
| 環境 | 静的な障害物 | センサ・コンベア・軸を持ち、工程に応じて**振る舞う** |
| 著作物 | Motion(教示点列) | Sequence(工程 + 遷移条件)+ Motion |
| 成果物 | JointTrajectory | SequenceTimeline — **サイクルタイム**、工程帯、信号波形、オブジェクト軌跡 |
| 渡し先 | CSV / JSON / URScript | + USD アニメーション(usdview / Omniverse / Blender で再生) |

`examples/sequence_demo.py` がこれを実演する: コンベア搬送 → ビームセンサ検出 → ベルトを止めずに追従把持 → パレットへ搬送。13 工程・**サイクルタイム 20.61s**(停止式なら 23.31s)が決定的に焼き上がり、そのまま USD になる。サイクルタイムは設備の現場で最も見られる数字であり、1 本の軌道の所要時間とは価値の階層が違う。**botrail が売るのは「セル 1 サイクルが成立することの保証と、その数字」である。**

### 1.2 なぜ「環境の自由度」が武器になるのか — プランナの役割の転換

既存のオフラインプログラミングツール(RoboDK、Visual Components、Process Simulate)では、動作は人が明示的に教示する。だから**レイアウトを 10 cm 動かすと教示が全部壊れる**。セルが「一度作ったら触りたくない資産」になる主因はこれである。

botrail では教示点と工程だけを書けば、**動作は計画され、時間は計算結果として出てくる**。コンベア速度を変えても、パレットを動かしても、`simulate_sequence()` を呼び直せばセルは再び成立する — あるいは成立しないことが即座に分かる。

> **プランナがあるから、環境を自由に動かしてもセルが壊れない。だから環境をデータとして、セルをコードとして扱える。**

USD import が「環境を自由に入れられる」を、シーケンサが「環境が振る舞う」を、プランナが「環境を変えても壊れない」を担保する。3 つが揃って初めて意味を持つ構図であり、プランナは商品ではなくなった代わりにコンセプトの土台になった。

### 1.3 4 本の柱

| 柱 | 意味 | 実装での裏付け |
|---|---|---|
| **環境がデータである** | セルは USD から入り、ファイルとして持ち運べる | USD import(reference/variant/instancing、cm→m・Y-up→Z-up 正規化)、名前付きフレーム、`.botrail` の可搬プロジェクト |
| **環境が振る舞う** | 環境は静的な障害物ではなく、センサ・機器・工程を持つ | PLC 語彙のシーケンサ(SFC 工程歩進 + 信号 + スキャンサイクル)、Zone/Beam センサ、Conveyor/LinearAxis、コンベアトラッキング |
| **決定的に焼ける** | 同じ入力からは常に同じタイムラインが出る | rollout の決定性(2 回焼いて bit 一致をテストで保証)、サイクルタイム、タイミングチャート、タイムラインのアサーション API(`step_span` / `signal` / `min_clearance`)とセル回帰テスト |
| **成果物が開いている** | 誰のツールにも渡せる形式で出る | USD アニメーション書き出し(pxr で独立検証済み)、USD 録画の再生、CSV/JSON、URScript、Python コード生成 |

**5 本目(構想): セルは URL で渡せる。** wasm でスタジオが静的配信できるのは botrail 固有の性質であり、競合はすべてインストールとライセンスを要するデスクトップアプリである。セルを 1 つの URL にして「開いて再生を押せばサイクルが動く」状態にできれば、これは構造的に真似されない配布・提案動線になる(§9)。

この 4 本を成立させる前提として、初期からの 3 原則は不変である。

1. **軽量導入** — 依存ゼロの wheel を pip install するだけ。システムライブラリ・ビルド不要、GPU 不要。
2. **UI ファースト** — 環境・制約・モーション・工程を 3D UI 上で作り込み、コードと往復できる。可視化は「デバッグ用のおまけ」ではなく第一級のワークフロー。
3. **どこでも動く** — コアは Rust。Python バインディング(pyo3)と wasm ビルド(ブラウザ完結)の両方を提供する。

### やらないこと(non-goals)

- **物理シミュレーション(動力学・接触)** — 幾何 + 運動学 + 離散スキャンに徹する。**決定性はこの割り切りの直接の果実**であり、セルを diff・テスト・CI に載せられる根拠そのものなので、ここは崩さない。
- **離散事象の生産シミュレーション**(スループット、バッファ、AGV、人の動線、統計) — FlexSim / Plant Simulation の領域。乱数と統計の世界に入ることは、決定性という最大の武器を自ら捨てることを意味する。botrail は「工場シミュレータ」ではなく **「セル検証器」** である: 1 サイクルの成立性・所要時間・干渉・到達性に賭ける。
- **CAD(STEP/JT)の直接 import** — 入口は USD に賭け、変換は外部ツールの仕事とする。
- ROS との統合レイヤの提供(ユーザー側で軌道を publish するのは自由)
- v1 での実機ドライバ実装(プラグインインターフェースの定義まで)
- モバイルロボット・多脚などのプランニング(将来検討)

## 2. ポジショニング(既存ツールとの比較)

比較対象は 2 層に分かれる。上のレイヤ(セルを組んで検証する)が現在の主戦場であり、下のレイヤ(ROS-free モーションプランニング)は botrail がそこに立つための土台である。

### 2.1 上のレイヤ — セル・オーサリング / 検証

| ツール | 導入 | セル環境 | 動作 | 成果物 |
|---|---|---|---|---|
| RoboDK | 商用・デスクトップ | CAD / 独自 | 明示教示中心 | 実機プログラム(ポスプロが豊富) |
| Visual Components | 商用・デスクトップ | 独自コンポーネント | 教示 + コンポーネント挙動 | レイアウト検証・生産シミュレーション |
| Process Simulate / DELMIA | 商用・PLM 統合 | CAD | 教示 | 工程検証(導入が重い) |
| Isaac Sim | 無償だが GPU 必須・大型 | **USD ネイティブ** | 物理 + 学習 | USD / 合成データ |
| Gazebo / Webots / CoppeliaSim | OSS | 独自 or SDF | 物理 | ROS トピック等 |
| **botrail** | **pip / ブラウザ** | **USD import** | **計画 + PLC 工程** | **USD / CSV / 実機スクリプト** |

上 3 つはセルを組む語彙を持つが閉じており、下 3 つは開いているが**工程の語彙を持たない**(ロボット単体の物理検証が主眼で、コンベア・センサ・工程歩進は各自スクリプトで書く)。

**空白地帯: 「セルを開いた形式(USD + Python)で組み、決定的に検証し、開いた形式で渡す」OSS は存在しない。** 商用 OLP / シミュレータのセルはプロプライエタリなプロジェクトファイルの中にあり、diff も CI も原理的に不可能である。botrail のセルはテキスト(Python / `.botrail` JSON / USD)であり、git で管理でき、同じ入力からは常に同じタイムラインが焼ける。**レイアウト変更に対するサイクルタイムの回帰テストという運用は、現状どのツールでも成立しない。** これが botrail 固有の主張である。

Isaac Sim との関係は競合ではなく補完である: Isaac の資産(USD ロボット・セル)をそのまま読み、書き出した USD は Omniverse で開ける。「Isaac 資産は持っているが、レイアウト検討とサイクル検証のために毎回 Isaac Sim を立ち上げるのは重い」という穴を埋める。

### 2.2 下のレイヤ — ROS-free モーションプランニング

| ツール | 導入 | UI | 備考 |
|---|---|---|---|
| MoveIt | ROS 必須・重い | RViz プラグイン | 機能は最も豊富 |
| Tesseract | C++/一部 ROS | 簡易ビューア | ROS-Industrial 系 |
| mplib | pip 1発 | なし | pinocchio+OMPL+FCL。コンセプトが近いが UI なし |
| pyroboplan | pip | なし | 教育向け、pinocchio ベース |
| Drake | pip(大きい) | meshcat(表示のみ) | 高機能だが学習コスト大 |
| cuRobo | NVIDIA GPU 必須 | なし | GPU 並列プランニング |
| Jacobi Robotics | 商用 | Studio(Web) | 発想が最も近いがクローズド |
| viser / meshcat / rerun | pip | Web(表示+gizmo) | 可視化のみ、プランニングなし |

この層でも **「pip 1発 + プランニング + 編集可能な 3D UI」を OSS で揃えたものは存在しない。** mplib(計画はあるが UI がない)と viser(UI はあるが計画がない)の間を埋める位置は変わらない。また xurdf 採用により **ROS なしで Xacro 形式のロボット記述をそのまま読める**ことも、実務ユーザーに対する地味だが効く差別化になる(実ロボットの記述は xacro が多く、他の ROS-free ツールはここで脱落しがち)。

### 2.3 想定ユーザーと入口

- **一次ユーザー**: USD 隣接のロボティクス開発者・SIer・研究/教育。Isaac / Omniverse の資産を持ちながら、レイアウト検討とサイクル検証を軽く回したい層。
- **入口**: wasm によるブラウザ完結デモ。導入前に価値を体験させる強力な入口であり、将来的にはセルそのものを URL で共有する動線になる。
- **語り口の二層構造**: 対外的な一行目は「USD ネイティブ・ROS フリーのセル・オーサリング」という具体で入り、概念の背骨は「決定的なセル検証」に置く。前者だけでは Isaac の脇役に見え、後者だけでは何ができるか伝わらない。

### 2.4 現時点の正直な穴

コンセプトを誇張しないために、この節を維持する。

- **協調計画なし・SFC は直列表記のみ** — マルチロボット(多台数設置・ロボット間干渉の tick 検証・ゾーンインターロック・2 台の studio 表示 / USD 書き出し)は 2026-08 の R0〜R4 で実装済みで、「厳密にはステーション」の但し書きは解除した([docs/design-multi-robot.md](design-multi-robot.md) §9)。正直に残る限界: 計画は他ロボットを現姿勢で凍結する(協調計画・swept volume 非対応 — 実行中干渉は tick 検証が時刻付きエラーで検出し、修正はユーザーのインターロック)、両腕同時把持(閉ループ)は不可、SFC の並列分岐**表記**は未提供(非同期起動 + `robot_done` で同等を書く。糖衣は需要待ち、同 §10)。
- **USD を持っている層は Isaac / Omniverse ユーザーに偏る** — 伝統的な設備屋は STEP の世界にいる。当面はそこを取りに行かないと割り切る。
- **PLC 語彙はメンタルモデルの互換であって実機接続ではない**(OPC-UA なし)。ここは誇張しない。実機との閉ループはベンダー I/O 出力(§9)で先に閉じ、制御論理の引き渡しも実機接続ではなく標準ファイルで行う構想 — シーケンスの PLCopen XML(IEC 61131-10)書き出し(§4.3・§4.6・§9)。
- **物理がないため**、部品の落下・安定、ケーブル、把持の滑りは検証できない。botrail が答えるのは「掴めるか」ではなく **「届くか・ぶつからないか・何秒か」** である。

## 3. 全体アーキテクチャ

```
                ┌───────────────────────────────────┐
                │      botrail studio (Web UI)      │
                │  TypeScript + React + R3F         │
                │  + three-usd-robot (USD 描画/FK)  │
                └────────────────┬──────────────────┘
                                 │ SessionBackend インターフェース
              ┌──────────────────┴──────────────────┐
              │ WebSocket RPC                       │ in-process (wasm)
    ┌─────────┴─────────┐               ┌───────────┴───────┐
    │  Python プロセス   │               │  botrail-wasm     │
    │  botrail (pyo3)   │               │  (wasm-bindgen)   │
    └─────────┬─────────┘               └───────────┬───────┘
              └──────────────────┬──────────────────┘
                    ┌────────────┴────────────┐
                    │     botrail-session     │  メッセージディスパッチ
                    │  (ホスト差分 = SessionHost) │  (両バインディング共通)
                    └────────────┬────────────┘
        ┌────────────────────────┴────────────────────────┐
        │  botrail-core (Rust)                            │
        │  model / kin / collide / plan / traj             │
        │  scene (obstacles / motion / seq / rollout)      │
        │  usd (import / export / recording) · export      │
        └─────────────────────────────────────────────────┘
```

### 設計上の最重要ポイント

- **UI はバックエンドを知らない。** UI は `SessionBackend`(シーン取得・変更適用・計画要求・軌道受信)という TypeScript インターフェースにのみ依存し、実装として (a) WebSocket RPC(Python サーバモード)と (b) wasm 直接呼び出し(ブラウザ完結モード)の 2 つを持つ。これで UI を一度書けば両モードで使い回せる。
- **メッセージディスパッチは 1 箇所。** hub(pyo3)と wasm で鏡写しになりがちな処理は `botrail-session` に集約し、ホスト差分(scene アクセス・emit・時計・ログ・アセット URL)だけを `SessionHost` trait として注入する。**過去に「片方だけ直してもう片方が壊れる」バグを複数回踏んでおり**、この集約はその再発防止でもある。
- **シーン・軌道・コマンドはすべて serde でシリアライズ可能な型として core に定義する。** プロトコルスキーマは Rust の型(`botrail-scene/src/wire.rs`)が唯一の真実で、TypeScript 型は ts-rs で自動生成する。
- **Python API と UI は同じ操作モデルを共有する。** 「UI でできることはすべて API でできる」を不変条件とし、UI 操作は内部的に同じ wire メッセージに落ちる。
- **コアは USD を知らない。** `botrail-usd` は境界クレートであり、Scene が受け取るのは正規化済み(m・Z-up)の障害物・フレーム・`RobotModel` である。書き出しも「Scene + 関節軌道 + オブジェクトトラック」を受けて USD を著作する片方向の依存になっている。これによりコアの型が USD に汚染されず、wasm ビルドでも同じ境界がそのまま成立する。
- **USD ロボットのリンク名は prim パス**(three-usd-robot との naming contract)。サーバとクライアントが同じ識別子でリンクを指すため、衝突ハイライト・ゴールゴースト・録画再生のいずれも対応表なしで 1:1 に解決する。

### Rust ワークスペース構成

```
crates/
  botrail-model     # URDF/Xacro パース(xurdf)、キネマティックツリー、関節・リミット
  botrail-mesh      # メッシュ読み込み(STL/OBJ、bytes 入力で wasm 対応)
  botrail-kin       # FK / ヤコビアン / IK(減衰最小二乗+リミット考慮+null-space)
  botrail-collide   # parry3d ベース。自己干渉+環境干渉、距離クエリ、ACM、VHACD 凸分解(rayon 並列は feature)
  botrail-plan      # サンプリングベース計画(RRT-Connect、決定的シード)、ランダムショートカット
  botrail-traj      # 時間パラメタライズ(IPTP 系)、Hermite サンプリング、リサンプリング
  botrail-scene     # ワールド + 著作。lib.rs(障害物/フレーム/attach/センサ/デバイス)、
                    #   motion.rs、seq.rs(シーケンス型)、rollout.rs(スキャンエンジン)、
                    #   wire.rs(プロトコル型・ts-rs)、project.rs(.botrail + Python 生成)
  botrail-usd       # USD 境界。lib.rs(シーン import)、articulation.rs(ロボット import)、
                    #   export.rs(アニメーション書き出し)、recording.rs(録画読み込み)
  botrail-export    # ベンダースクリプトエクスポート(中間表現+ポストプロセッサ。URScript 実装済み、TM/AUBO/DENSO は今後)
  botrail-session   # メッセージディスパッチ(両バインディング共通。ホスト差分は SessionHost trait)
  botrail-py        # pyo3 バインディング + WebSocket/静的ファイルサーバ
  botrail-wasm      # wasm-bindgen バインディング
  botrail-bench     # 衝突・距離クエリのベンチ(docs/bench-parry3d.md)
studio/             # Web UI (TypeScript, React + react-three-fiber + three-usd-robot)
python/             # Python パッケージング(maturin)、高レベル API(seq ビルダ)、studio バンドル同梱
```

### エコシステム方針: 汎用部品は独立パッケージに切り出す

ロボティクス一般に再利用可能な部品は、botrail 本体に抱え込まず独立クレート/パッケージとして開発・公開する。分割の基準は **「botrail のデータモデル(Scene/Motion)を知らなくても使えるか」** — 知らなくても使えるものは外に出す。

- **xurdf**(外部・既存)— URDF + Xacro パーサ。ROS ランタイム非依存で xacro(property/macro/include/if 等)を扱える点は botrail の差別化にも直結する。nalgebra ベースで botrail の数学スタックとも一致。
- **three-usd-robot**(外部・既存)— studio 側の USD ロボット/ステージ描画とクライアント FK。botrail のデータモデルを知らない純粋な表示ライブラリであり、この方針の実例になっている(botrail 側の要求から `worldUp` オプションや `setLinkTransforms` の表示モード契約が入った)。
- **openusd**(外部・固定 rev)— USD の読み書き。`botrail-usd` はこの上の薄い変換層に徹する。
- **メッシュ I/O クレート**(未着手)— 当初は STL/OBJ/DAE の独立クレートとして切り出す計画だったが、`botrail-mesh` として同梱のまま STL/OBJ 止まりである。**DAE 対応は未着手**で、高忠実度の表示経路は USD import + three-usd-robot が担うようになった。DAE が要るのは URDF 系ロボットの visual メッシュであり、需要が出たときに再開する。凸分解は現状 `botrail-collide` 側のランタイム機能(キャッシュ付き)として持っている。

## 4. データモデル

概念は **ワールド / 著作物 / 焼き込み結果 / 保存形式** の 4 種に整理される。

```
Scene ──┬── Motion   ──(plan)──────→ PlannedMotion / Trajectory   1 本の動作
        └── Sequence ──(simulate)──→ SequenceTimeline             1 サイクルのセル
                                          ↓
                             再生 / USD 書き出し / CSV / 検証
        Project (.botrail) = Scene + Motion 群 + Sequence 群 + アセット
```

**「著作物 → 焼き込み結果」の対が 2 組ある**のが要点で、両者は常に分けて持つ。著作物(教示点列・工程列)は所要時間に依存せず安定であり、焼き込み結果は計画とシミュレーションの計算結果として絶対時刻を持つ。ユーザーが編集するのは常に前者で、後者は再生・書き出し・検証が消費する。モーション所要時間は再計画のたびに変わるため、著作の側に時刻を持たせないことが「環境を動かしてもセルが壊れない」(§1.2)の前提になっている。

### 4.1 Scene — ワールド

ロボットは base pose 付きでワールドに設置され、Scene の入出力(リンク姿勢・IK ターゲット・制約・障害物)はすべてワールド座標。住人は以下の 8 種:

| 要素 | 内容 |
|---|---|
| **Robot** | `RobotModel`(URDF / Xacro / USD 由来 — `RobotSource` enum)+ base pose + 関節値 |
| **Obstacle** | box / sphere / cylinder / mesh。`enabled` で衝突判定と計画妥当性から除外できる(表示は残る)。USD 由来なら名前は prim パス |
| **Frame** | 名前付きワールド姿勢。設置点・教示参照であり衝突対象ではない |
| **Attachment** | 把持状態 `{ object, link, grasp: link←object の固定相対姿勢, touch_links }`。attach 中の障害物は環境集合から外れてリンク随伴集合に入り、計画・衝突・再生・書き出しのすべてで随伴する(MoveIt の attached collision object 相当) |
| **ACM** | 許容衝突行列。隣接ペア + サンプリングによる常時衝突ペアの自動検出 |
| **Sensor** | Zone(直方体の在荷/エリアセンサ)/ Beam(光電センサ = 細いカプセル)。`watch` で監視対象を絞る。**センサ名がそのまま読み取り専用の入力信号名になる** |
| **Device** | Conveyor(ゾーン内の非 attach 障害物を等速搬送)/ LinearAxis(指定オブジェクト群の軸方向位置決め — 扉・リフタ・ストッパ・インデクサ) |
| **SignalDef** | 内部信号(PLC の内部リレー M)の宣言 |

Sensor / Device / SignalDef は **環境に振る舞いを与える住人**であり、Sequence からは名前で参照される(§1.3「環境が振る舞う」)。物理は解かず、センサは形状交差クエリ、コンベアはゾーン所属則で決める — 擬似だが決定的で頑健であり、これが §1.3「決定的に焼ける」の土台になる。

MoveIt の SRDF に相当する情報(グループ、エンドエフェクタ、デフォルト ACM)は botrail 独自の軽量フォーマットで持ち、**UI 上のセットアップウィザードで生成できるようにする**(MoveIt Setup Assistant の体験を Web で置き換える)。ACM 自動生成はコア実装済み、ウィザード UI は未実装。

### 4.2 Motion / PlannedMotion — 1 本の動作

- **Motion** — 名前付きのセグメント列。各セグメント = `{ ゴール(関節値 or TCP 姿勢), 計画方式(joint-space plan / cartesian line), 制約 }`。「教示点を並べてモーションを作る」という現場のメンタルモデルに合わせる。
- **PlannedMotion / Trajectory** — 計画結果。時間パラメタライズ済みの関節軌道(`JointTrajectory` = times / positions / velocities + Hermite サンプリング)を全セグメント連結で 1 本持ち、`segment_ends` で境界時刻を、`segments` でセグメントごとの疎なウェイポイント列(スクリプトエクスポータが消費)を保持する。各セグメント境界は静止する。
- **軌道キャッシュは未実装** — 計画は要求のたびに走る。シーケンスも「毎回 `simulate()` で焼き直す」を基本とする(決定的なので、後から工程単位のキャッシュを足しても結果は変わらない)。

### 4.3 Sequence / SequenceTimeline — 1 サイクルのセル

**Sequence(著作物)** — PLC の工程歩進(IEC 61131-3 の SFC、三菱のステップラダー、OMRON の工程歩進命令として設備業界で共有されているメンタルモデル)に写像した工程列。各 `Step` は「突入時に発行するアクション」+「次工程への遷移条件」を持つ。v1 は直列(分岐なし)で、これは**タイムラインが一意に焼けること = USD 書き出しと検証の前提**を守るための制約である。

| PLC の語彙 | botrail での対応物 |
|---|---|
| 入力接点(X) | Sensor の評価結果(読み取り専用) |
| 出力コイル(Y) | `Action::Device`(コンベア ON/OFF、軸 MoveTo) |
| 内部リレー(M) | `SignalDef` + `Action::Set` |
| タイマ(TON) | `Condition::Elapsed` |
| ロボットへの起動指令 + 完了信号 | `Action::StartMotion` + `Condition::Done` |
| 位置決め完了(インポジション) | `Condition::DeviceDone` |
| スキャンサイクル(入力→演算→出力) | rollout の Δt tick(既定 10 ms) |
| タイミングチャート | studio の TimelineDock(工程帯 + 信号波形レーン) |

アクションは他に `StartRamp`(汎用関節ランプ — グリッパ開閉をジョイントグループ概念なしで賄う)、`Attach` / `Detach`(把持・開放)、`Track` / `Untrack`(コンベアトラッキング — 教示姿勢を動くパーツに乗せ、ラインを止めずに拾う)。条件は他に `Immediately` / `Signal`(レベル評価)/ `All`(直列接点 = AND)/ `Any`(並列接点 = OR)。

**IEC 61131-3 への整列を設計制約とする(2026-08 決定)。** botrail のシーケンサが 61131-3 の 5 言語のうち対応するものは **SFC** であり、ST は遷移条件・アクションを記述する式の層としてだけ使う(フル ST = 走査周期の処理系とベンダー FB の再実装はやらない)。SFC には標準交換形式 **PLCopen XML(IEC 61131-10)** が存在するので、その SFC 語彙(step / transition / divergence / actionBlock、遷移条件は ST 式)へ**無損失に写像できること**を Sequence モデルの設計制約として維持する。写像は現行モデルで既に素直に成立している:

| botrail | PLCopen XML(SFC) |
|---|---|
| `Step`(アクション + 遷移条件) | `<step>` + `<actionBlock>` + `<transition>` |
| `Signal` / `All` / `Any` | 遷移条件の ST 式(`AND` / `OR` / `NOT`) |
| `Elapsed` | ステップ経過時間 `StepName.T >= T#5s` |
| `Done` / `RobotDone` / `DeviceDone` | ハンドシェイク用 BOOL 変数(VAR 宣言込みで書き出す) |
| `StartMotion` / `Attach` / `Track` 等 | N 修飾子のアクションが呼ぶスタブ FB(`FB_StartMotion` 等 — 実 PLC 側で実装に置換) |
| 並列分岐(将来の糖衣、§2.4) | `simultaneousDivergence` に受け皿あり |

この制約は PLCopen XML 書き出し(§4.6・§9)の前提であると同時に、PLC 語彙が設備業界の標準から漂流しないためのガードレールでもある。

**SequenceTimeline(焼き込み結果)** — `simulate()` の出力。同じ Scene + Sequence からは常に **bit 一致**のタイムラインが焼ける(テストで保証)。

```rust
SequenceTimeline {
    duration,      // = サイクルタイム
    robot,         // JointTrajectory(待機区間はホールド)
    objects,       // ObjectTrack: 動いたオブジェクトのみ
    signals,       // BoolTrack: センサ・内部信号・デバイス状態の波形
    step_spans,    // 各工程の [開始, 終了] + 名前
}
```

`ObjectTrack` を **シンボリック区間**(`Hold` / `Follow { link, offset }` = 把持随伴 / `Linear { from, velocity }` = 搬送)で持つのが要点。任意レートで厳密に再サンプルできるため、30 Hz の studio 配信・60 fps の USD 書き出し・8 ms の CSV がすべて同一ソースから誤差なく出る。

**再生・USD 書き出し・CSV・アサーションはすべてこの型だけを消費する。** 逆に USD 録画の読み込み(`import_recording`)も同じ形に落とすので、Isaac Sim の録画と botrail 自身の焼き込みが同じ経路で再生される。

アサーション層(2026-08 実装)は `tl.step_span(name)`(工程の締切)・`tl.signal(name)`(エッジ・レベル・ON 区間の波形クエリ)・`tl.min_clearance()` の 3 面。クリアランスは焼き込み時に記録せず、**タイムライン+焼き込み時スナップショットからの再走査**で測る — タイムラインは純粋な運動記録のまま保たれ、サンプリングレートは呼び出し側の選択になる。追従把持のティックや搬送物は rollout 自身が衝突検証しない領域であり、この再走査が唯一の計測になる(接触時は最初の接触時刻と接触ペアを報告する)。

### 4.4 Project (`.botrail`)

Scene + Motion 群 + Sequence 群 + アセットを束ねた保存形式。バージョンフィールド付きの JSON(現行 v2)で、メッシュや USD ステージへの参照があるときだけ zip(`project.json` + `assets/`)になり、ロード時はコンテンツアドレスのキャッシュへ展開する — **元ファイルを消しても再ロードできる**のが可搬性の条件。

拡張は **additive** に行う運用が確立している(version probe + `#[serde(default)]` + tagged enum): `frames` / `sequences` / `signals` / `sensors` / `devices` はすべて v2 のまま後から足された。ロボットは `robots: [{ name, source, base_pose, joint_positions }]` の配列で持ち、複数台の保存/読込/Python 生成に対応済み(2026-08、単一ロボットの出力は従来とバイト互換 — `name` も additive)。

**UI で作ったプロジェクトを Python から `bt.Scene.load_project()` で読んで実行時利用する**のが主要な往復動線。あわせて `generate_python()` が「現在の状態を再現する Python スクリプト」を出力し、ここも全要素(障害物・フレーム・attach・シーケンス・センサ・デバイス)に対応する — セルがテキストとして出てくることが §2.1 の「git で管理できる」の実体である。

### 4.5 制約(v1 スコープ)

- 関節リミット(位置・速度・加速度)
- 姿勢制約(ツール軸のコーン制約 — 「コップを傾けない」)
- 位置領域制約(TCP を box 領域内に保つ)
- パス全体 or ゴールのみへの適用を選択可能

### 4.6 成果物の射影 — USD と PLCopen XML の棲み分け

エクスポートは「一つの著作から、**受け手のエコシステムが標準にしている語彙**への射影」として整理する。焼き込みとは遷移条件を評価して分岐や待ちを一本の実現されたタイムラインへ潰す操作なので、**USD(焼き込みの出力)と PLCopen XML(焼き込みの入力)は同じものの別表現ではなく、相互に代替不可能な補完**である — USD には論理(条件・分岐 — 焼き込みで失われる情報そのもの)が乗らず、PLCopen XML には幾何・軌道が乗らない。

| 出力 | 層 | 運ぶもの | 受け手 |
|---|---|---|---|
| USD | 焼き込み結果 | 幾何 + タイムライン(1 実現値) | 3D エコシステム(usdview / Omniverse / Blender)、レビュー・承認する人 |
| CSV / URScript | 焼き込み結果 | うちロボット軌道 | ロボットコントローラ |
| PLCopen XML(構想、§9) | 著作物 | うちシーケンス論理(SFC) | PLC IDE(Beremiz / CODESYS)、制御担当 |
| Project / `generate_python()` | 著作物 | セル定義の全体 | botrail 自身(再現・再編集) |

USD の customData にシーケンス構造を埋め込むことはしない — 読めるのが botrail だけでは「成果物が開いている」(§1.3)ことにならず、著作の保存形式は Project という権限分離を崩す理由もない。論理の開放性は IEC 61131-10 経由で果たす。なお仮想試運転側の容器規格 AutomationML(IEC 62714)は「PLCopen XML(論理)+ COLLADA(幾何)」を束ねる構成であり、botrail の USD + PLCopen XML の対はその現代版に相当する — 将来デジタルファクトリー側とブリッジする際の接点はここになる。

## 5. UI(botrail studio)機能スコープ

### 5.1 実装済み

| 領域 | パネル / コンポーネント | 内容 |
|---|---|---|
| 3D ビューポート | `Viewport` / `SceneView` / `UsdRobotView` / `WasmStageView` | ロボット表示(URDF はサーバ FK、USD は three-usd-robot でクライアント FK)、クリックで TCP フォーカス、フォーカスチップ、メッシュ/USD のドラッグ&ドロップ読み込み |
| 姿勢操作 | `TcpGizmo` / `TcpPanel` / `JointPanel` / `RobotBaseGizmo` / `RobotPanel` | TCP gizmo ドラッグ → リアルタイム IK(到達不可/干渉は色で警告)、関節スライダ、ベース配置とフレームへのスナップ |
| レイアウト編集 | `ObstacleView` の `GroupGizmo` / `SceneTreePanel` / `store.drillChain` | **インポートしたサブツリーを 1 剛体として**移動・回転。ビューポートのクリックは**1 回目が機械、もう 1 回で部品**(`/World/Pedestal` → `/World/Pedestal/Column`)で、ステージルート(全障害物がその下にある階層)は選択対象から外す — 「全部」を選ぶのはクリックの意味ではないため。シーンツリーからは任意の階層を直接選べる。**教示フレームも一緒に動く** — 機械だけ動かしてフレームを置き去りにするのは静かに壊れたセルなので。ドラッグは `update_poses`(バッチ)1 通で送り、4px 未満の移動だけをクリックとみなす(ドラッグ終了時に部品へ降りてしまうため) |
| シーン編集 | `ObstaclePanel` / `ObstacleView` / `SceneTreePanel` | 障害物の追加・gizmo 移動・寸法編集・削除、attach/detach(🧲 バッジ)、prim パス階層のツリー、表示 👁 と衝突有効/無効のトグル、フレームの ⌖ 配置 |
| モーション | `MotionPanel` / `PlanPanel` / `GhostRobot` | ウェイポイント列の編集、ゴール設定(ゴースト表示)、計画要求 → 結果軌道、クリアランス表示 |
| シーケンス | `SequencePanel` | 工程リスト(アクションチップ + 遷移)、プリセット追加(Motion / Wait / Grasp / Release)、simulate |
| タイムライン | `TimelineDock` / `PlaybackDriver` | 工程帯 + 信号波形レーン(タイミングチャート)、サイクルタイム、クリックシーク、**世界再生**(ロボット + 把持物 + 搬送物を同一クロックで) |
| センサ/デバイス | `SensorView` / `SceneTreePanel` | ゾーンの半透明ボリューム / ビームロッド、ON ハイライト、一覧行 |

### 5.2 未実装(v1 スコープに残っているもの)

- **セットアップウィザード** — URDF 読み込み → グループ定義 → 自己干渉サンプリングによる ACM 生成。**ACM 自動生成はコア実装済みで、UI だけがない**(MoveIt Setup Assistant の体験を Web で置き換える構想)。
- **コード生成の UI 導線** — `generate_python()` はコアにあるが studio にボタンがない。「UI→コードの往復」は Python 側からのみ成立している。
- **関節速度/加速度グラフ** — 軌道プレビューは再生・シーク・ゴースト・干渉ハイライトまで。
- **USD 書き出しのダウンロード** — Python からは `export_usd`。wasm は `export_to_string` があるので導線を足せば成立する(§9)。
- **センサ/デバイスの作成フォーム** — 定義は Python 側を正とし、studio は一覧表示中心。
- **プロジェクトの保存/読込 UI** — `.botrail` の入出力も現状 Python 側のみ。

### 5.3 技術選定

TypeScript + React + react-three-fiber + zustand。USD ロボット/ステージの描画とクライアント FK は three-usd-robot。通信は WebSocket 上の JSON(必要になったら MessagePack へ。計測してから)で、型は Rust から ts-rs 生成。

## 6. Python API スケッチ

```python
import botrail as bt

# ---- ロボットとセル環境 --------------------------------------------
robot = bt.Robot.from_usd("franka.usd")             # from_urdf / from_xacro も
scene = bt.Scene(robot)
scene.load_usd("factory.usda", prefix="env")        # 障害物 + 名前付きフレーム
scene.set_robot_base_pose(*scene.frame("env/World/MountFrame"))
scene.add_box("table", size=(0.6, 0.6, 0.05), position=(0.4, 0.0, 0.0))
scene.set_obstacle_color("table", (0.43, 0.25, 0.10))  # 表示のみ(linear RGB)

bt.studio(scene)                                    # ブラウザで studio を開く

# ---- 触る・計画する -------------------------------------------------
scene.set_tcp_target((0.3, 0.1, 0.5))               # ライブ IK(studio に反映)
scene.in_collision(), scene.min_obstacle_distance()
traj = scene.plan_to_pose((0.4, 0.1, 0.3))          # IK → RRT-Connect → 時間パラメタライズ
traj.export_csv("pick.csv", dt=0.008)
traj.export_script("pick.script", dialect="urscript")

# ---- 環境に振る舞いを与える -----------------------------------------
scene.add_conveyor("conv", zone_position=(-0.45, 0.62, 0.60),
                   zone_size=(1.3, 0.4, 0.14), velocity=(0.15, 0.0, 0.0))
scene.add_beam_sensor("beam_pick", frm=(0.55, 0.42, 0.60), to=(0.55, 0.82, 0.60),
                      watch=["/World/Conveyor/Box_A"])
scene.define_signal("carrying")

# ---- 工程を書く(PLC の工程歩進)------------------------------------
sq = scene.sequence("pick_place")
sq.step("feed",  actions=[bt.seq.start("conv"), bt.seq.motion("to_hover")],
                 transition=bt.seq.all_of(bt.seq.signal("beam_pick"), bt.seq.done()))
sq.step("latch", actions=[bt.seq.track(BOX)])       # ベルトを止めずに追従
sq.step("grasp", actions=[bt.seq.attach(BOX, link="/panda/panda_hand"),
                          bt.seq.set_signal("carrying")])
sq.step("carry", actions=[bt.seq.untrack(), bt.seq.motion("to_pallet")])
# 遷移を省略すると、動作があれば done、なければ immediately が補われる

# ---- 焼く・渡す -----------------------------------------------------
tl = scene.simulate_sequence("pick_place")          # 決定的
print(tl.duration)                                  # = サイクルタイム
for step, t0, t1 in tl.step_spans: ...              # 工程帯
tl.export_usd("cell_anim.usda", fps=60)             # usdview / Omniverse / Blender で再生

# ---- 往復 -----------------------------------------------------------
scene.save_project("cell.botrail")                  # メッシュ/USD 資産を同梱
scene2 = bt.Scene.load_project("cell.botrail")
print(scene.generate_python())                      # 現在の状態を再現するスクリプト
scene.play_usd_animation("cell_anim.usda")          # 録画(Isaac Sim 由来でも可)を studio で再生
```

実機連携は v1 ではインターフェース定義のみ:

```python
class TrajectoryExecutor(Protocol):
    def execute(self, traj: bt.Trajectory) -> None: ...
```

## 7. 技術選定と主要リスク

| 領域 | 選定 | リスク・検証事項 |
|---|---|---|
| 数学 | nalgebra | 低リスク(xurdf と共通) |
| URDF/Xacro | xurdf | 自作クレートのため拡張が容易。mimic joint 等の対応範囲と package:// のメッシュパス解決は botrail 要件に合わせて確認・拡張 |
| メッシュ読込 | `botrail-mesh`(同梱、STL/OBJ) | 当初計画の「独立クレート化 + DAE 対応」は**未着手**。高忠実度の表示経路を USD import + three-usd-robot が担うようになったため優先度が下がった。DAE が要るのは UR 系など DAE visual を持つ URDF ロボットで、需要が出たら再開する(§3 エコシステム方針) |
| 衝突判定 | parry3d | **ベンチ検証済み([docs/bench-parry3d.md](bench-parry3d.md))。** ブール判定は 61 ペアのフルシーンで ~5µs と十分速い。ただし TriMesh 同士は包含を衝突として検出せず距離も誤るため、衝突形状はメッシュを VHACD 凸分解(ロード時 ~1s/メッシュ+キャッシュ)した compound に統一する。parry ≥0.23 は glam ベースになったため botrail-collide 境界で nalgebra⇔glam 変換を行う |
| IK | 自前(減衰最小二乗+null-space) | TRAC-IK 級の成功率にはリスタート戦略等の作り込みが必要。まず gizmo 追従に十分な品質を優先 |
| プランナ | 自前 RRT-Connect + shortcut | OMPL 移植ではなく自前実装。まず 1 本を堅牢に |
| 時間パラメタライズ | TOTG(Kunz & Stilman)系を自前実装 | jerk 制限(Ruckig 相当)は将来 |
| Python | pyo3 + maturin(abi3 wheel) | 低リスク。manylinux/mac/Windows の CI 整備 |
| wasm | wasm-bindgen | rayon 並列は SharedArrayBuffer/COOP-COEP が必要。**v1 の wasm はシングルスレッドで割り切る** |
| 通信 | WebSocket + JSON(serde) | 型は ts-rs で TS へ自動生成し、スキーマ二重管理を避ける |

## 8. マイルストーン

体験の核である「触って動かせる」に最短で到達する順序にする。各マイルストーンはデモ可能な状態で締める。

- **M0: 骨格**(~2週)**[完了]** — ワークスペース scaffold、xurdf による URDF/Xacro 読み込み+FK、pyo3 バインディング、three.js で robot 表示(WebSocket で関節状態配信)。`bt.studio(scene)` でブラウザにロボットが出る。メッシュは STL/OBJ から。ts-rs による protocol.ts 自動生成も導入済み。
- **並行トラック: メッシュ I/O クレート**(**未着手**)— 独立パッケージ化と DAE 対応を M2 までに、という計画だったが実施していない。`botrail-mesh` が同梱のまま STL/OBJ を担い、高忠実度の表示は USD 経路に移った(§7)。
- **M1: 触れる**(~2週)**[完了]** — ヤコビアン+DLS IK(特異点ストール脱出付き)、TCP gizmo ドラッグでリアルタイム IK、到達可否フィードバック。**ここが最初の「刺さる」デモ。**
- **M2: ぶつかる**(~3週)**[完了]** — 自己干渉+環境干渉(botrail-collide、ソリッド形状規約)、contact ベース距離クエリ、障害物編集 UI(追加/選択/gizmo 移動/寸法編集/削除)、衝突ハイライト+クリアランス表示、ACM(隣接+サンプリングによる常時衝突ペア自動検出)。メッシュ衝突形状は実装済み(botrail-mesh + VHACD 凸分解、コンテンツハッシュのディスクキャッシュ付き。リンク・障害物とも対応)。セットアップウィザード UI は未実装(ACM 自動生成はコア実装済み)。
- **M3: 計画する**(~3週)**[完了]** — RRT-Connect(決定的シード・妥当性コールバック方式)+ランダムショートカット、IPTP 系時間パラメタライズ(速度/加速度制限、Hermite サンプリング)、`scene.plan()`/`plan_to_pose()` + JSON/CSV エクスポート、studio のゴール設定(ゴースト表示)・計画要求・軌道再生(再生/シーク)。ジャーク制限(Ruckig 相当)と複数セグメントのモーション列は M4 以降。
- **M4: 作り込める**(~3週)**[完了]** — Motion(ウェイポイント列)エディタ、cartesian line セグメント(シード連続 IK 追従+構成ジャンプ検出)、制約(姿勢コーン/位置ボックス、妥当性フィルタ方式 — 射影ベースの狭い制約対応は将来)、`.botrail` 保存/読込(URDF 埋め込みの自己完結 JSON、メッシュ資産の同梱はメッシュ I/O 待ち)、JSON/CSV エクスポート、Python コード生成。UI⇄コードの往復動線が成立。
- **P3(USD シーン import)[完了]** — botrail-usd クレート(openusd を git rev 固定、コアは USD を知らない境界を維持)。usda/usdc/usdz + composition(reference/variant/instancing)を読み、可視・default purpose のジオメトリをワールド座標の障害物として抽出(cm→m・Y-up→Z-up 正規化、スキーマ fallback 値表、`omniverse://` は search path へ読み替え)。メッシュはコンテンツハッシュ名の STL としてキャッシュに実体化し、既存の VHACD 衝突・`/meshes` 配信・studio 表示をそのまま通す。葉の Xform/Scope は名前付きフレーム(設置点、姿勢は共役変換で「Y-up の identity は Z-up でも identity」)になり、`scene.load_usd()` / `scene.frame()` / `scene.add_frame()` として Python に公開。project v2 に frames を追加(additive)。既知の残課題: 大規模シーン初回の VHACD が直列で遅い(Kitchen_set 1.7k メッシュで数十分、キャッシュ後は数秒)→ 並列分解 or 遅延分解を P4 以降で。three-usd-robot による高忠実度表示は P5。
- **P4(USD articulation import)[完了]** — `bt.Robot.from_usd()`: `PhysicsArticulationRootAPI` サブツリーの UsdPhysics joint(Revolute/Prismatic/Fixed、world アンカーは基底固定として解釈)を URDF 流ツリーへ変換。核となるフレーム変換は「モデルの子リンクフレーム = ジョイントフレーム」で、origin = K_parent⁻¹∘localPose0、ボディ配下のジオメトリは K⁻¹(K = 自身の localPose1)で再表現 — URDF 双子モデルとの FK 一致テストで検証。度→ラジアン、metersPerUnit、Y-up 共役変換、`physxJoint:maxJointVelocity`(deg/s)、DriveAPI maxForce に対応。リンク/ジョイント名は prim パス(three-usd-robot との naming contract)。`RobotModel.source` は `RobotSource` enum になり、project は USD ロボットをパス参照で往復(`load_project` が再インポート)。既知の制限: mimic/テンドン非対応、閉ループ reject、Capsule 近似なし(警告)、SphericalJoint 等はスキップ。
- **P5(wire v2 + three-usd-robot 描画統合)[完了]** — `SceneInit` に `usd_asset`(URL + articulation root)を追加し、USD ロボットは studio が **three-usd-robot でクライアント側レンダリング + FK**。`TrajectoryMsg.link_poses` は Option 化し、USD ロボットの軌道再生は **joint 値のみ**(サーバ 30Hz FK 焼き込みなし)。衝突ハイライトは `highlightLink`、ゴールゴーストは `createGhostRobot`(いずれも prim パスの naming contract で 1:1 対応)。サーバは `/assets/*` でロボットのステージディレクトリを配信(相対参照解決、パストラバーサル拒否)。ホスト差分は `SessionHost::robot_asset_url`(wasm はデフォルト None → レガシー経路)。レガシー URDF ロボットは従来経路のまま無変更。
- **P6(フレーム UI + プロジェクト同梱)[完了]** — フレームを wire に公開(`frames` メッセージ、ハンドシェイクは scene_init/obstacles/motions/frames/state の 5 通)し、studio の Robot パネルに「place at frame」スナップ配置を追加(クライアントは既存の `set_robot_base_pose` を送るだけ)。`.botrail` はメッシュ参照があるときだけ zip(`project.json` + `assets/`)として保存され、ロード時はコンテンツアドレスのキャッシュに展開して URL を書き換える — 元ファイルを消しても再ロード可能(可搬性テストあり)。メッシュなしプロジェクトは従来どおり素の JSON(magic sniff で両対応)。
- **P7(残課題前半)[完了]** — (1) **VHACD 並列化**: botrail-collide の `parallel` feature(rayon)で リンク構築とバッチ障害物追加を並列化。`Scene::add_obstacles` はコライダを全構築してから挿入する atomic 動作に。botrail-py のみ有効化、wasm は非依存(cargo tree で確認)。実測: Kitchen_set 1,788 メッシュのフレッシュキャッシュ import が 1 時間超 → **9 分**。(2) **wasm USD ドロップ**: `ImportOptions::meshes_in_memory`(メッシュを `usd:/<prim>` 仮想パス + `MeshData` で保持、fs 書き込みなし)+ `import_usd_bytes`(単一レイヤ/usdz の in-memory リゾルバ)+ `ObstacleCollider::from_shape` / `Scene::add_obstacle_with_collider` で、`WasmSession::load_usd_scene(bytes)` が成立。studio は wasm モードでビューポートに USD をドロップ → セッションに衝突・フレーム登録 + three-usd-robot(`loadSceneGeometry`)でステージを直描画。**ブラウザでの実機視認は未実施**(コンパイル・単体テスト・デモビルドまで)。
- **P8(残課題後半)[完了]** — (1) **シーンツリーパネル**: 障害物/フレーム名(prim パス)から階層を構築。行ごとに 👁(クライアント側の表示トグル)+ 衝突チェックボックス(新設 `Obstacle.enabled` — wire `set_obstacle_enabled`、無効時は衝突判定・計画妥当性から除外。フィルタ後インデックスのズレは remap で対処し回帰テストあり)+ フレームの ⌖ 配置ボタン。(2) **USD ロボットの project 同梱**: `stage_dependencies()`(stage を全トラバースして `layer_identifiers()` を収集)でレイヤ群を `robot/<relpath>` として zip に同梱、ロード時にキャッシュへ展開して再インポート(元ステージ削除後の再ロードをテストで検証)。ステージディレクトリ外のレイヤは警告して絶対パス参照のまま。(3) **ブラウザ実機視認完了**: USD ロボットの three-usd-robot 描画、joint スライダ→クライアント FK、Set goal→ゴースト、Plan→**joint 値のみでの再生アニメーション**、フレームスナップ(base が (0.1,0,0.4) に一致)、衝突トグル(clearance 45mm↔316mm)を Chrome 上で確認。**発見したバグ 2 件を修正**: `/assets` ルートが vite のバンドル出力 `/assets/*` を遮蔽して SPA が白画面(→ `/usd-assets` に改名)、STL 書き出しの法線ゼロで黒シェーディング(→ 面法線を書く + ローダ側 computeVertexNormals)。
- **P9(残課題完結)[完了]** — (1) **wasm VHACD の Web Worker 化**: `decompose_usd_scene`(worker 内の別 wasm インスタンスで composition + VHACD → hull 点集合 JSON)+ `WasmSession::load_prepared_scene`(メインスレッドで hull → compound 再構築のみ、軽量)。フォールバックとして同期経路も維持。(2) **wasm ドロップ実機視認**: Chrome 上で合成 DragEvent によりドロップ → Worker 経由の取り込み → シーンツリー/障害物/clearance/ステージ描画まで確認。発見バグ 2 件を修正: `std::env::temp_dir()` が wasm で panic(キャッシュディレクトリ既定値を遅延解決に)、three-usd-robot の軸規約とのズレは v0.8.1 で解決(`worldUp` オプション追加。botrail は両ローダで `worldUp: "Z"` を明示 — **v0.8.1 の既定は "Y" なので明示必須**。手動 +90°X 補正は撤去、prepared JSON の up_axis はメタデータとして残置)。(3) **Isaac 公式アセットゴールデン照合**: 公開 S3 ミラーから Franka/UR10 を取得し照合 — **Franka 9 DOF・リミットが公式スペック(±2.8973 等、指 0–0.04m)と厳密一致、UR10 6 DOF ±2π 一致**。テストは `BOTRAIL_ISAAC_DIR` 指定時のみ実行(URSim と同方式)。重大バグを発見・修正: nalgebra `Rotation3::from_matrix` は**反復無制限**で、Franka 指リンクの悪条件行列で無限ループ → `from_matrix_eps(…, 100, …)` に変更。既知の残り: franka.usd の visual メッシュは外部参照(instanceable)のためローカルに無く、衝突は関節のみで幾何なし — アセットパックを `search_paths` に与えれば解決する設計。
- **P10(デモ整備)[完了]** — `examples/demo.py` を Franka + 工場セルの USD デモに刷新。(1) ロボットは Isaac 公式 Franka(`franka.usd` + `Props/panda_*.usd` 12 ファイル + `Materials/Materials.usd`、計約 10MB)を初回実行時に公開 S3 ミラーから `~/.cache/botrail/assets/franka/` へダウンロード(`BOTRAIL_CACHE_DIR` 準拠)。Props を相対レイアウトのまま置くことで参照が解決し、**visual メッシュ込みで import・studio 描画とも完全動作**(P9 の既知課題「franka visual 外部参照」はこれで解消 — search_paths 不要)。(2) 環境は手書きの `examples/assets/factory.usda`(Z-up・m 単位、床・台座・コンベア・パレット・棚・安全柵 + MountFrame/PickFrame/PlaceFrame の 3 フレーム。当初 36 プリム、A1 で 115 プリムの実セルに書き直し)。全部 Cube/Cylinder なので VHACD 不要で即ロード。台座はロボット基部と 10mm クリアランス(基部 VHACD ハルが基準面下に ~2mm 膨らむため 5mm では HUD が 3.2mm 張り付きになる)。(3) ブラウザ実機確認済み: Franka 実メッシュ描画、ready ポーズ、ギズモ IK、Set goal → Plan → 再生(2.25s / 35ms)。発見した仕様上の注意: **プランナは関節値が上限値ちょうど(指 0.04m 等)のゴールを「out of limits」で弾く**(境界を排他扱い)— デモは 0.035 で回避、境界包含にするかは要検討。追補: ロボット描画(USD/URDF とも)にクリックハンドラを追加 — R3F はハンドラ無しメッシュをピッキングで素通しするため、アームをクリックすると背後の障害物が選択されてしまっていた(クリックで TCP フォーカス+stopPropagation)。あわせてホバーで pointer カーソル、ビューポート左下に現在フォーカス(TCP リンク/障害物名/robot base)のチップを常設。
- **M5: 広める**(~2週)**[実装完了]** — botrail-wasm(wire プロトコル互換の `WasmSession`、v1 方針どおりシングルスレッド)+ studio の `SessionBackend` 抽象(WebSocket / wasm の 2 実装、UI 無変更)+ ブラウザ完結デモビルド(`scripts/build_wasm_demo.sh`、静的配信で全機能動作を検証済み)。GitHub Pages デプロイは**稼働済み(2026-08-02)**: https://neka-nat.github.io/botrail/ でブラウザ完結デモが配信されている(Pages ソースを GitHub Actions に設定して発火。初回失敗の原因は Pages サイト未作成による deploy-pages の 404)。PyPI リリースは workflow 整備済みで、Trusted Publisher 設定後に発火する。ドキュメントサイトは未着手。hub/wasm で鏡写しだったメッセージディスパッチは botrail-session クレートに集約済み(ホスト差分は `SessionHost` trait — scene アクセス・emit・時計・ログ — として注入)。
- **S1〜S6(セル化)[完了 2026-07]** — (S1) attach/detach と把持物込みの計画、(S2) USD アニメーション書き出し、(S3) PLC 型シーケンスコア(工程歩進 + 信号 + スキャンサイクル)、(S4) 擬似センサ・デバイス、(S5) USD 録画再生(Isaac Sim の録画も botrail 自身の書き出しも同じ経路)、(S6) コンベアトラッキング。設計と実装記録は [docs/design-sequence-control.md](design-sequence-control.md)。**これにより成果物の単位が「1 本の軌道」から「1 サイクルのセル」へ移った**(§1.1)。
- **A1(見た目)[完了 2026-08-02]** — 障害物の**表示色をパイプラインに通した**。それまで `primvars:displayColor` は import 時に捨てられ、studio は全障害物を同一グレー、書き出し USD も `ENV_COLOR` 一色だったため、セルがどれだけ作り込まれていても粘土模型に見えていた。(1) importer が `displayColor` を読む。**単一要素(= constant)のみ**を採用し、それを名前空間の下方向に継承させる — グループ Xform 1 つでサブツリー全体を塗れる。複数要素は per-vertex データなので、先頭要素を平坦色として採用すると作者が指定していない色を発明した上に子へ漏れるため、レンダラに委ねる。(2) `Obstacle.color: Option<[f32;3]>` を新設し、wire(`ObstacleMsg.color`)・プロジェクト保存・USD 書き出し・Python(`add_*(color=)` / `set_obstacle_color` / `obstacle_color`)まで一本で通した。**衝突・計画には一切影響しない表示専用**。(3) studio: 色が付いた障害物は「作者が仕上げた景色」として不透明+影を落とす、色が無いものは従来どおり半透明のコリジョンプロキシ(ロボットが透けて見える性質を残す)。あわせて shadow map(soft)と `RoomEnvironment` による IBL を導入 — Isaac Franka は本物の metalness/roughness を持つのに envMap が無く artificially dull だった。HDRI をネットから取らない手続き生成なのでオフラインでも動く。(4) `examples/assets/factory.usda` を 36 → 115 プリムの実セルへ書き直し(コンベア側枠・ローラ・脚・ギヤモータ・透過形センサ、EUR パレット、パレットラック、制御盤+積層signal tower、床のライン)。**色は linear で書く**(USD 規約。concrete #6b6e73 は 0.147/0.156/0.171)。計画コストは 115 障害物でも `check_collisions` 0.17ms / `plan` 17ms で悪化なし。
- **A2(ブラウザデモの実機化)[完了 2026-08-02]** — GitHub Pages のデモは `simple_arm.urdf` 1 本+空環境だった(USD をドロップして初めて何か出る)。これを **Python デモと同じ Franka + 工場セル**にした。障壁は 2 つあり、どちらも本体側の穴だったので埋めた。(1) **複数レイヤの合成**: `franka.usd` は `Props/panda_*.usd` 等 14 ファイルの参照で構成されるが、wasm 用の `MemoryResolver` は 1 レイヤしか配れなかった。`BundleResolver`(`name -> bytes` のマップ、`./` `../` とスキーム+ホストを畳んで正規化)+ `import_robot_bundle` を追加。openusd の `Resolver` は同期なのでブラウザ内で fetch できない → **呼び出し側が先に落として bytes で渡す**契約にした。(2) **メッシュのファイル依存**: ロボットインポータは常に STL をキャッシュに書き出しており、ファイルシステムの無い wasm で動かなかった。`RobotImportOptions::meshes_in_memory`(ジオメトリのパスを `usd:/<prim>` にして三角形は `ImportedRobot::meshes` で返す)と、`botrail_collide::mesh` 側の**メモリ内メッシュ登録**(`register_memory_mesh`、`load_mesh_compound` がファイルより先に参照)を追加。これで `Geometry::Mesh` の消費側(ロボットリンクのコライダ含む)が無改造で動く。配信方針: **Franka は NVIDIA の CDN から直接取得**する(バケットが `Access-Control-Allow-Origin: *` を返すことを確認済み)。Pages の成果物に 10MB の第三者アセットを載せず、再配布も発生しない(なおライセンスは Apache-2.0 なので載せること自体は可能)。取得に失敗した環境では同梱の `simple_arm` にフォールバックして studio は生きたままにする。セル `factory.usda` だけは自前なので成果物に同梱。ブラウザ実機確認済み: Franka の Isaac マテリアル描画、115 障害物 + 3 フレーム、`MountFrame` への設置と ready ポーズ、干渉なし。テストは `BOTRAIL_ISAAC_DIR` 指定時のみ走る 2 本(バンドル取り込みがファイル経路と一致すること、メモリ内メッシュ由来のコライダが実際に衝突を検出すること)。既知の無害な 404: three-usd-robot が `OmniPBR.mdl` を投機的に取りに行くが公開バケットに無い(内蔵の OmniPBR 解釈にフォールバックし描画は正常)。

**成功条件(v1・軌道粒度)**: 新規ユーザーが `pip install botrail` から 5 分以内に、URDF/Xacro/USD のロボットで障害物を避ける pick モーションを UI 上で作成し、CSV エクスポートまで到達できる。

**成功条件(セル粒度)**: USD のセル環境を読み込み、工程を並べて `simulate_sequence()` を回すとサイクルタイムが出て、`export_usd()` した USD が usdview / Omniverse で再生される。**`examples/sequence_demo.py`(13 工程・20.61s)で成立済み。** 次の水準「レイアウトを変えて再 simulate し、サイクルタイムの差分が CI で検出される」も 2026-08 に自リポジトリで成立: `python/tests/test_cell_regression.py` がサイクルタイム・工程順・信号ハンドシェイク・クリアランスを assert し、ビームセンサ移設 +0.25 m → サイクル +1.0 s の差分検出をテストする(§9)。

## 9. 将来ロードマップ(v1 以降)

- 最適化ベースプランナ(TrajOpt/CHOMP 系)によるパス品質向上
- jerk 制限付き時間パラメタライズ(Ruckig 相当)
- スクリプトエクスポートのバックエンド追加(TMScript / AUBO Lua / DENSO PAC — 実機検証できる協力者がいるものから)と Python カスタムバックエンドのプラグイン化、studio からのダウンロード(botrail-export は wasm でも動く純文字列生成)
- **ベンダー I/O 出力** — export IR に SetDigitalOut / WaitDigitalIn / Sleep を追加し、シーケンスから I/O 付きの実機プログラムを生成する。検証したセルと実機プログラムが同一ソースから出る = 検証ループが閉じる(§2.4)
- **IEC 61131-3 整列の段階実装(SFC + ST 式 + PLCopen XML)** — §4.3 の写像制約を実利に変える 4 段: ① SequencePanel の遷移条件を ST 記法で**表示**(パーサ不要、コスト極小)→ ② `when("beam_pick AND NOT carrying")` 形式の **ST 式入力**(既存 Condition への小さなコンパイル。決定性そのまま)→ ③ シーケンスの **PLCopen XML(SFC POU)書き出し**(§4.6) — Beremiz / OpenPLC / CODESYS で開ける標準ファイルとして制御担当へ渡す(スタブ FB 方式。OSS ツールで開けるため書き出しの検証も CI で閉じられる)→ ④ **取り込み**サブセット(構造 + ST 式のみ、座標は無視、ベンダー FB はスタブ)は需要が見えてから。普及の重心は CODESYS 圏 + OSS であり、三菱/オムロン等の国産 IDE がネイティブに読む形式ではないことは誇張しない。フル ST 処理系と実 PLC 接続(OPC-UA)はやらない(§2.4)。着手順はセルのアサーション API が先で、ST 式は著作 API を次に触るときに同乗させる
- 実機プラグイン(UR / xArm 等 1 機種からリファレンス実装)
- 物理シミュレータ連携(MuJoCo 等への軌道再生エクスポート)
- マルチロボット / デュアルアーム — **実装完了(2026-08、R0〜R4)**: 複数設置・per-robot 計画(他ロボットは凍結体)・ロボット間衝突の tick 検証・多アクターシーケンス(`robot_done` インターロック)・studio 複数表示・USD 書き出し/録画の per-robot 化まで一気通貫([docs/design-multi-robot.md](design-multi-robot.md) §9 に各フェーズの実装記録)。調停はインターロックで、並列 SFC なしで成立(同 §3.3)。残タスクは需要待ちの未決のみ(同 §10: default_robot 設定、SFC 並列の糖衣表記、studio ロボット追加 UI)
- **セルのアサーション API と CI 統合** — **実装完了(2026-08)**: `SequenceTimeline` に `step_span(name)` / `signal(name)`(rising/falling エッジ・レベル・ON 区間)/ `min_clearance(dt)`(スナップショット再走査 — §4.3。float として比較でき、接触時は repr が時刻とペアを名指しする)。自リポジトリ CI のセル回帰テスト `python/tests/test_cell_regression.py`(ゴールデンサイクル ±0.25s・工程順・ハンドシェイク・クリアランス床・レイアウト変更→差分検出)まで一気通貫。README にも「Verify the cell, not just the trajectory」節として提示済み
- **パラメトリック・セル / パラメータスイープ** — 実例あり(2026-08): `examples/sweep_demo.py` がベルト速度×レーン位置の 2 軸を振り、「速度はサイクルだけを動かし、レーンはクリアランスだけを削る」を決定的な表として出す。API としての sweep ヘルパ(並列焼き込み等)は需要待ち
- **セルの URL 共有** — wasm の `export_to_string` + studio からのダウンロード/読み込みで、セルを 1 つのリンクとして渡す(§1.3 の 5 本目)
- wasm マルチスレッド化

## 10. 開発運用

- monorepo(crates/ + studio/ + python/)、ライセンスは MIT
- CI: cargo test / wheel ビルド(maturin-action, Linux/mac/Windows)/ wasm ビルド / studio のユニット+Playwright スモーク
- URScript エクスポートの実機シミュレータ検証: `scripts/ursim_test.sh`(docker の URSim を起動し、エクスポートしたスクリプトを流して関節ストリームで到達を検証。`BOTRAIL_URSIM_HOST` 未設定時はテストは自動スキップ)
- 単位・規約: SI(m, rad)、右手系 Z-up、クォータニオンは (x, y, z, w) — **初期に決めてドキュメント化**
- デモ駆動: 各マイルストーンの成果を GIF/動画にして README に置く。ブラウザ完結デモが最良のマーケティング

## 11. 未決事項(実装しながら決める)

- プランニンググループ定義フォーマットの詳細(SRDF サブセット互換にするか独自か)
- メッシュ I/O クレートの名前とスコープ(読み込みのみか、glTF 変換・凸分解・簡略化まで含むか)
- 凸分解をランタイムに含めるか、メッシュクレート側のオフライン機能として切り出すか
- studio のセッション同時接続(複数ブラウザ)の扱い — v1 は単一クライアント想定でよい
- npm パッケージとしての studio 単体配布(viser 的な利用)をするか
- IEC 整列(§9)の詳細 — ST 式パーサの文法範囲(エッジ検出 `R_TRIG` 相当を式に含めるか)、PLCopen XML 書き出しの検証水準(XSD 検証まで / Beremiz・matiec を CI に入れて「開ける」まで確認するか)
