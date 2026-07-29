# botrail 設計方針

> ROS 非依存・UI ファーストのロボットモーション作成ライブラリ

## 1. ビジョン

**「`pip install botrail` と数行のコードだけで、ブラウザ上の 3D UI でロボットの環境構築・制約設定・モーション作成ができる」**

MoveIt が解いている問題(FK/IK・衝突回避・軌道計画)は多くのロボット開発で必要だが、その導入コスト(ROS 環境、launch/パラメータ設定、SRDF 生成、ビルド環境)は「アームを 1 台動かしたい」ユーザーには過剰である。botrail は次の 3 点を核とする。

1. **軽量導入** — 依存ゼロの wheel を pip install するだけ。システムライブラリ・ビルド不要。
2. **UI ファースト** — 環境(障害物)・制約・モーションを 3D UI 上で作り込み、コードと往復できる。可視化は「デバッグ用のおまけ」ではなく第一級のワークフロー。
3. **どこでも動く** — コアは Rust。Python バインディング(pyo3)と wasm ビルド(ブラウザ完結)の両方を提供する。

### やらないこと(non-goals)

- ROS との統合レイヤの提供(ユーザー側で軌道を publish するのは自由)
- 物理シミュレーション(動力学・接触)— 幾何と運動学に徹する
- v1 での実機ドライバ実装(プラグインインターフェースの定義まで)
- モバイルロボット・多脚などのプランニング(将来検討)

## 2. ポジショニング(既存ツールとの比較)

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

**botrail の空白地帯: 「pip 1発 + プランニング + 編集可能な 3D UI」を OSS で揃えたものは存在しない。** mplib(計画はあるが UI がない)と viser(UI はあるが計画がない)の間を埋める。wasm によるブラウザ完結デモは、導入前に価値を体験させる強力な入口にもなる。また xurdf 採用により **ROS なしで Xacro 形式のロボット記述をそのまま読める**ことも、実務ユーザーに対する地味だが効く差別化になる(実ロボットの記述は xacro が多く、他の ROS-free ツールはここで脱落しがち)。

## 3. 全体アーキテクチャ

```
                    ┌─────────────────────────────┐
                    │   botrail studio (Web UI)   │
                    │  TypeScript + three.js/R3F  │
                    └──────────┬──────────────────┘
                               │ SessionBackend インターフェース
              ┌────────────────┴────────────────┐
              │ WebSocket RPC                   │ in-process (wasm)
    ┌─────────┴─────────┐             ┌─────────┴─────────┐
    │  Python プロセス   │             │  botrail-wasm     │
    │  botrail (pyo3)   │             │  (wasm-bindgen)   │
    └─────────┬─────────┘             └─────────┬─────────┘
              └────────────────┬────────────────┘
                    ┌──────────┴──────────┐
                    │  botrail-core (Rust) │
                    │  model / kin / collide / plan / traj / scene
                    └─────────────────────┘
```

### 設計上の最重要ポイント

- **UI はバックエンドを知らない。** UI は `SessionBackend`(シーン取得・変更適用・計画要求・軌道受信)という TypeScript インターフェースにのみ依存し、実装として (a) WebSocket RPC(Python サーバモード)と (b) wasm 直接呼び出し(ブラウザ完結モード)の 2 つを持つ。これで UI を一度書けば両モードで使い回せる。
- **シーン・軌道・コマンドはすべて serde でシリアライズ可能な型として core に定義する。** プロトコルスキーマは Rust の型が唯一の真実で、TypeScript 型は自動生成する(ts-rs 等)。
- **Python API と UI は同じ操作モデルを共有する。** 「UI でできることはすべて API でできる」を不変条件とし、UI 操作は内部的に API 呼び出しに落ちる。

### Rust ワークスペース構成

```
crates/
  botrail-model     # URDF/Xacro パース(xurdf)、キネマティックツリー、関節・リミット
  botrail-kin       # FK / ヤコビアン / IK(減衰最小二乗+リミット考慮+null-space)
  botrail-collide   # parry3d ベース。自己干渉+環境干渉、距離クエリ、ACM
  botrail-scene     # ワールドモデル: 障害物(primitive/mesh)、attach/detach、フレーム
  botrail-plan      # サンプリングベース計画(RRT-Connect → 拡張)、パス簡略化
  botrail-traj      # 時間パラメタライズ(TOTG 系)、スプライン、リサンプリング
  botrail-export    # ベンダースクリプトエクスポート(中間表現+ポストプロセッサ。URScript 実装済み、TM/AUBO/DENSO は今後)
  botrail-mesh      # メッシュ読み込み(STL/OBJ、bytes 入力で wasm 対応)。DAE/glTF は importer 層へ
  botrail-session   # 操作モデル(コマンド適用・undo/redo・イベント配信)。両バインディングの共通層
  botrail-py        # pyo3 バインディング + WebSocket/静的ファイルサーバ
  botrail-wasm      # wasm-bindgen バインディング
studio/             # Web UI (TypeScript, React + react-three-fiber)
python/             # Python パッケージング(maturin)、高レベル API、コード生成
```

### エコシステム方針: 汎用部品は独立パッケージに切り出す

ロボティクス一般に再利用可能な部品は、botrail 本体に抱え込まず独立クレート/パッケージとして開発・公開する。URDF/Xacro パーサの xurdf がその先例で、同じ方針を他にも適用する。

- **xurdf**(既存)— URDF + Xacro パーサ。ROS ランタイム非依存で xacro(property/macro/include/if 等)を扱える点は botrail の差別化にも直結する(実ロボットの記述は xacro 形式が多い)。nalgebra ベースで botrail の数学スタックとも一致。
- **メッシュ I/O クレート**(新規)— STL/OBJ/DAE を統一メッシュ構造体に読み込む独立クレート(Rust + wasm 対応、必要なら Python バインディングも)。Rust エコシステムで DAE 対応が薄いという穴を埋めるもので、単体でも価値がある。botrail 側の io 層はこれの薄いラッパにする。
- 凸分解(衝突用メッシュ前処理)も、実装する場合はこのメッシュクレート側のオフライン機能として持たせる選択肢がある。

分割の基準: 「botrail のデータモデル(Scene/Motion)を知らなくても使えるか」。知らなくても使えるものは外に出す。

## 4. データモデル

概念は 4 つに絞る。**Scene / Motion / Trajectory / Project**。

- **Scene** — ワールド中心: ロボット(URDF 由来のモデル+プランニンググループ定義)は base pose 付きでワールドに設置され、Scene の入出力(リンク姿勢・IK ターゲット・制約・障害物)はすべてワールド座標。障害物(box/sphere/cylinder/mesh)、フレーム、attach 状態、ACM(許容衝突行列)。プロジェクト形式 v2 はロボットを `robots: [{source, base_pose, joint_positions}]` の配列で持ち(コードは当面 1 台を強制)、マルチロボット拡張と USD 由来ロボット(`source` の enum 追加)を additive に受けられる。MoveIt の SRDF に相当する情報(グループ、エンドエフェクタ、デフォルト ACM)は botrail 独自の軽量フォーマットで持ち、**UI 上のセットアップウィザードで生成できるようにする**(MoveIt Setup Assistant の体験を Web で置き換える)。
- **Motion** — 名前付きのセグメント列。各セグメント = `{ ゴール(関節値 or TCP 姿勢), 計画方式(joint-space plan / cartesian line), 制約, プランナ設定 }`。「教示点を並べてモーションを作る」という現場のメンタルモデルに合わせる。
- **Trajectory** — 計画結果。時間パラメタライズ済みの関節軌道(サンプル列+区分多項式)。Motion にキャッシュされ、シーン変更で無効化される。
- **Project (`.botrail`)** — Scene + Motion 群 + アセット(メッシュ)を束ねた保存形式。JSON(+アセットは zip 同梱)、バージョンフィールド付き。**UI で作ったプロジェクトを Python から `bt.Project.load()` で読んで実行時利用する**のが主要な往復動線。

### 制約(v1 スコープ)

- 関節リミット(位置・速度・加速度)
- 姿勢制約(ツール軸のコーン制約 — 「コップを傾けない」)
- 位置領域制約(TCP を box 領域内に保つ)
- パス全体 or ゴールのみへの適用を選択可能

## 5. UI(botrail studio)機能スコープ

v1 で提供する画面要素:

1. **3D ビューポート** — ロボット表示、TCP ゴールの transform gizmo ドラッグ → リアルタイム IK 追従(到達不可/干渉時は色で警告)
2. **シーン編集** — 障害物の追加・gizmo による移動/回転/スケール、メッシュのドラッグ&ドロップ読み込み、オブジェクトの attach/detach
3. **モーションエディタ** — ウェイポイントのリスト/タイムライン、セグメントごとの計画方式・制約の設定、「計画」ボタン → 進捗表示 → 結果軌道
4. **軌道プレビュー** — スライダ再生、ゴースト表示(スイープ確認)、干渉箇所のハイライト、関節速度/加速度グラフ
5. **セットアップウィザード** — URDF 読み込み → グループ定義 → 自己干渉サンプリングによる ACM 生成
6. **コード生成** — 現在のプロジェクトを再現する Python スクリプトのエクスポート。UI→コードの往復を保証する

技術選定: TypeScript + React + react-three-fiber + zustand。通信は WebSocket 上の JSON(必要になったら MessagePack へ。計測してから)。

## 6. Python API スケッチ

```python
import botrail as bt

robot = bt.Robot.from_urdf("ur5e.urdf")           # メッシュも解決
scene = bt.Scene(robot)
scene.add_box("table", size=(1.2, 0.8, 0.05), pose=bt.Pose(z=-0.03))

# UI を起動してシーン/モーションを編集(ブラウザが開く)
bt.studio(scene)                                   # ブロッキング or 非同期どちらも可

# --- あるいはコードだけで ---
goal = robot.ik(bt.Pose(x=0.4, y=0.2, z=0.3, quat=...), seed=robot.home)
traj = scene.plan(
    start=robot.home,
    goal=goal,
    constraints=[bt.OrientationCone(axis="z", angle=0.2)],
)
traj.export_json("pick.json")
traj.export_csv("pick.csv", dt=0.008)

# --- UI で作ったプロジェクトの実行時利用 ---
proj = bt.Project.load("cell.botrail")
traj = proj.motion("pick_and_place").plan()        # シーン変更があれば再計画
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
| メッシュ読込 | 独立クレートとして新規開発(STL/OBJ/DAE) | botrail とは別パッケージで開発する並行トラック。M0 は STL/OBJ から始め、DAE 対応を M2 までに揃える(UR 系など主要 URDF の visual メッシュは DAE が多い) |
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
- **並行トラック: メッシュ I/O クレート** — M0 と並行して独立パッケージとして開発開始。M2 までに DAE 対応を入れ、botrail-io から差し替える。
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
- **P10(デモ整備)[完了]** — `examples/demo.py` を Franka + 工場セルの USD デモに刷新。(1) ロボットは Isaac 公式 Franka(`franka.usd` + `Props/panda_*.usd` 12 ファイル + `Materials/Materials.usd`、計約 10MB)を初回実行時に公開 S3 ミラーから `~/.cache/botrail/assets/franka/` へダウンロード(`BOTRAIL_CACHE_DIR` 準拠)。Props を相対レイアウトのまま置くことで参照が解決し、**visual メッシュ込みで import・studio 描画とも完全動作**(P9 の既知課題「franka visual 外部参照」はこれで解消 — search_paths 不要)。(2) 環境は手書きの `examples/assets/factory.usda`(Z-up・m 単位、床・台座・コンベア・パレット・棚・安全柵の 36 プリミティブ + MountFrame/PickFrame/PlaceFrame の 3 フレーム)。全部 Cube/Cylinder なので VHACD 不要で即ロード。台座はロボット基部と 10mm クリアランス(基部 VHACD ハルが基準面下に ~2mm 膨らむため 5mm では HUD が 3.2mm 張り付きになる)。(3) ブラウザ実機確認済み: Franka 実メッシュ描画、ready ポーズ、ギズモ IK、Set goal → Plan → 再生(2.25s / 35ms)。発見した仕様上の注意: **プランナは関節値が上限値ちょうど(指 0.04m 等)のゴールを「out of limits」で弾く**(境界を排他扱い)— デモは 0.035 で回避、境界包含にするかは要検討。追補: ロボット描画(USD/URDF とも)にクリックハンドラを追加 — R3F はハンドラ無しメッシュをピッキングで素通しするため、アームをクリックすると背後の障害物が選択されてしまっていた(クリックで TCP フォーカス+stopPropagation)。あわせてホバーで pointer カーソル、ビューポート左下に現在フォーカス(TCP リンク/障害物名/robot base)のチップを常設。
- **M5: 広める**(~2週)**[実装完了]** — botrail-wasm(wire プロトコル互換の `WasmSession`、v1 方針どおりシングルスレッド)+ studio の `SessionBackend` 抽象(WebSocket / wasm の 2 実装、UI 無変更)+ ブラウザ完結デモビルド(`scripts/build_wasm_demo.sh`、静的配信で全機能動作を検証済み)。GitHub Pages デプロイと PyPI リリースは workflow 整備済みで、リポジトリの push / Pages 有効化 / PyPI Trusted Publisher 設定後に発火する。ドキュメントサイトは未着手。hub/wasm で鏡写しだったメッセージディスパッチは botrail-session クレートに集約済み(ホスト差分は `SessionHost` trait — scene アクセス・emit・時計・ログ — として注入)。

**成功条件(v1)**: 新規ユーザーが `pip install botrail` から 5 分以内に、UR5e 相当の URDF で障害物を避ける pick モーションを UI 上で作成し、CSV エクスポートまで到達できる。

## 9. 将来ロードマップ(v1 以降)

- 最適化ベースプランナ(TrajOpt/CHOMP 系)によるパス品質向上
- jerk 制限付き時間パラメタライズ(Ruckig 相当)
- スクリプトエクスポートのバックエンド追加(TMScript / AUBO Lua / DENSO PAC — 実機検証できる協力者がいるものから)と Python カスタムバックエンドのプラグイン化、studio からのダウンロード(botrail-export は wasm でも動く純文字列生成)
- 実機プラグイン(UR / xArm 等 1 機種からリファレンス実装)
- 物理シミュレータ連携(MuJoCo 等への軌道再生エクスポート)
- マルチロボット / デュアルアーム(データモデルは複数ロボットを排除しない形で設計しておく)
- wasm マルチスレッド化

## 10. 開発運用

- monorepo(crates/ + studio/ + python/)、ライセンスは MIT OR Apache-2.0 のデュアル
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
