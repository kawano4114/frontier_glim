# frontier_glim: master からの変更と現在の動作

このリポジトリの ROS パッケージ名は `glim_ros` です。ROS 2 の入力、GLIM の処理パイプライン、TF・点群・姿勢の出力を担当します。大域最適化そのものは、隣の [glim リポジトリ](../glim/README_CUSTOM_CHANGES.md) が担当します。

本書は **2026-10-22 時点のソースと Git 履歴**に基づく説明です。現在の `trigger` ブランチの HEAD `cf0ba8e28b1c6465aa220e339930b85faabf75ca` に、次の 4 ファイルの staged 変更を加えた状態を対象にしています。これらの変更は HEAD だけを checkout しても含まれません。

- [include/glim_ros/glim_ros.hpp](include/glim_ros/glim_ros.hpp)
- [include/glim_ros/rviz_viewer.hpp](include/glim_ros/rviz_viewer.hpp)
- [src/glim_ros/glim_ros.cpp](src/glim_ros/glim_ros.cpp)
- [src/glim_ros/rviz_viewer.cpp](src/glim_ros/rviz_viewer.cpp)

本書の作成では、コードの編集、ビルド、ROS ノードや bag の実行はしていません。以下は実装から読み取れる動作であり、実機・シミュレーションでの動作確認結果ではありません。

## 1. 比較する「標準 master」と Git の関係

比較基準は、ローカルの `master` と取得済みの `origin/master` が指す **`4d4ec52`** です。`origin` は `https://github.com/kawano4114/frontier_glim.git` であり、上流本家の最新 master をネットワークから取得したものではありません。最新 upstream との一致は確認していません。

`master` は現在の `trigger` の祖先ではありません。共通祖先は `a62811dc3ab73076f4a43fc21005f96cd712903c` で、そこから `master` 側に 7 コミット、`trigger` 側に 11 コミットが分岐しています。そのため、master との全差分には、独自追加に加え **master 側だけにある更新の未取り込み**も含まれます。

```text
共通祖先 a62811d
  ├─ master 側の 7 コミット ── master / origin/master: 4d4ec52
  └─ 独自変更の 11 コミット ─ trigger: cf0ba8e ─ staged の修正
```

比較の再確認には次を使えます。`git diff master` は HEAD だけでなく現在の index・作業ツリーも含む比較です。

```bash
git merge-base master HEAD
git log --oneline master..HEAD
git log --oneline HEAD..master
git diff master --stat
git diff master -- src/glim_ros/glim_ros.cpp src/glim_ros/rviz_viewer.cpp
```

## 2. 編集箇所の全体像

| 箇所 | 変更内容 | どんなときに影響するか |
|---|---|---|
| [config/config_default](config/config_default)、[config/config_gazebo](config/config_gazebo) | ROS・センサー・前処理・Odometry・Sub Mapping・Global Mapping・Viewer・logging の設定セットを追加 | このリポジトリ内の設定を `config_path` に指定した起動時 |
| [launch/glim.launch.py](launch/glim.launch.py) | Gazebo 設定を使う launch を追加。ノード名 `glim_ros`、`use_sim_time=true` | 通常の `ros2 launch glim_ros glim.launch.py` 実行時 |
| [launch/rviz.launch.py](launch/rviz.launch.py)、[rviz/glim_ros.rviz](rviz/glim_ros.rviz) | RViz 起動と表示設定。全体地図を `/glim_ros/map` に統一 | RViz を同 launch で開くとき |
| [CMakeLists.txt](CMakeLists.txt) | `launch`・`rviz`・`config` を `share/glim_ros` へ install | install 済みパッケージから launch・設定を参照するとき |
| [package.xml](package.xml) | `std_msgs` 依存を追加 | Empty 購読のビルド・利用時 |
| [GlimROS のヘッダ](include/glim_ros/glim_ros.hpp)、[実装](src/glim_ros/glim_ros.cpp) | Empty 購読、モード別受付、ROS コールバックの寿命管理、終了時の待ち合わせ | 手動補正要求、コンポーネントのアンロード・終了時 |
| [RvizViewer のヘッダ](include/glim_ros/rviz_viewer.hpp)、[実装](src/glim_ros/rviz_viewer.cpp) | 全サブマップのスナップショット、独立した定期配信、合成キャッシュ、コールバック解除 | 全体地図を配信・再配信するとき |
| 旧 `submap_bridge`・`Submap.msg` | 一度追加した独自配信を撤去 | 旧 `/glim/submap`・`/glim/submap_cloud` を使っていた接続先には移行が必要 |
| 旧 `README.md` | `ed21720` で元の短い案内を削除 | 本書を独自変更の説明として参照する |

master との比較で見える `src/glim_rosbag.cpp`、`src/offline_viewer.cpp`、バージョン表記等の違いは、主に後述する master 更新の未取り込みによるものです。

## 3. 起動と設定セット

### 3.1 通常 launch が読む設定

```bash
ros2 launch glim_ros glim.launch.py
ros2 launch glim_ros rviz.launch.py
```

`glim.launch.py` は install 先の `share/glim_ros/config/config_gazebo` を絶対パスで `config_path` に渡します。ノードの名前は `glim_ros`、名前空間は指定していません。RViz の Fixed Frame は `map`、全体地図は `/glim_ros/map`、点群・Odometry・Pose は `/glim_ros/points`、`/glim_ros/odom`、`/glim_ros/pose` です。

この launch には設定セット切替の launch 引数がありません。別セットを使う場合は、例えば次のようにノードの `config_path` に絶対パスを指定します。

```bash
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/home/masaki/code/glim/frontier_glim/config/config_default \
  -p use_sim_time:=false
```

`config_path` が相対パスの場合、`GlimROS` は **glim パッケージの share ディレクトリ**から解決します。カレントディレクトリや `glim_ros` の share からの相対パスではありません。また、JSON 内の設定値は起動時読み込みであり、`ros2 param set` で `optimization_mode` や配信間隔を動的変更する実装はありません。

install 先の設定を使う launch に対してソース側の JSON だけを書き換えた場合、その変更が起動対象へ反映されているかは、利用する install 環境に依存します。

### 3.2 default と Gazebo の具体的な違い

両セットの [config.json](config/config_gazebo/config.json) は、既定で GPU 版 Odometry、GPU 版 Sub Mapping、GPU 用パラメータの Global Mapping を選択しています。CPU 用ファイルを編集しても、この選択を切り替えなければ通常 launch には反映されません。

| 設定 | `config_default` | `config_gazebo` | 動作上の意味 |
|---|---|---|---|
| `glim_ros.imu_topic` | `/os_cloud_node/imu` | `/imu/data` | 購読する IMU 入力 |
| `glim_ros.points_topic` | `/os_cloud_node/points` | `/points` | 購読する LiDAR 点群入力 |
| `glim_ros.acc_scale` | `0.0` | `1.0` | default は最初の加速度ノルムから単位を推定、Gazebo は入力値をそのまま使用 |
| `imu_frame_id` / `lidar_frame_id` / `base_frame_id` | 空欄 | `imu_link` / `lidar_link` / `base_link` | default はセンサーフレームを入力から取得。Gazebo は固定名 |
| `publish_imu2lidar` | `true` | `false` | Gazebo では GLIM から IMU–LiDAR の TF を出さない |
| `sensors.global_shutter_lidar` | `false` | `true` | Gazebo は点ごとの時刻をゼロ扱いにして deskew を無効化 |
| GPU Odometry の `smoother_lag` | `5.0` 秒 | `1.0` 秒 | 固定ラグ平滑化の保持窓 |
| GPU Sub Mapping の `max_num_keyframes` | `15` | `10` | サブマップに保持するキーフレーム上限 |
| GPU Sub Mapping の `keyframe_update_strategy` | `OVERLAP` | `DISPLACEMENT` | 重なり基準から変位基準へ変更 |
| GPU Sub Mapping の `keyframe_update_interval_trans` | `1.0` m | `0.1` m | 変位基準の並進しきい値 |
| GPU Sub Mapping の `submap_target_num_points` | `50000` | `10000` | サブマップ点群の目標点数 |

その他のファイル内容は両セットで同じです。共通設定として Local Mapping と Global Mapping は有効、CPU/GPU Global Mapping の `optimization_mode=3`、手動 LM 上限は `20`、全体地図配信間隔は `20.0` 秒です。GPU Sub Mapping 自体の `enable_optimization=false` は、Global Mapping の補正モードとは別設定です。

Gazebo 設定でも `T_lidar_imu` は default と同じ数値を保持しています。名前が Gazebo であるだけで任意の機体のセンサー配置に適合するわけではありません。`base_link` と `imu_link` が異なるため、この間の TF も必要です。`publish_imu2lidar=false` なのでセンサー間の TF はロボット側から供給する構成です。

## 4. Empty による大域補正要求

### 4.1 設定と責務

[config_ros.json](config/config_gazebo/config_ros.json) の `glim_ros.optimize_topic` で購読先を指定します。既定は **絶対名** `/glim_ros/optimize`、型は `std_msgs/msg/Empty`、購読 QoS は `rclcpp::QoS(10)` です。ノード名や名前空間を変えただけでは、この既定の絶対トピック名は変わりません。

```json
{
  "glim_ros": {
    "enable_local_mapping": true,
    "enable_global_mapping": true,
    "optimize_topic": "/glim_ros/optimize",
    "ros_pub_interval_time": 20.0
  }
}
```

上は編集するキーの抜粋です。既存の ROS 設定全体をこの断片だけに置き換えるものではありません。Global Mapping 側の設定例は次の通りです。

```json
{
  "global_mapping": {
    "so_name": "libglobal_mapping.so",
    "enable_optimization": true,
    "optimization_mode": 3,
    "manual_optimization_max_iterations": 20
  }
}
```

| 実効モード | 通常追加・定期処理 | Empty 受信 | 最適化を担当する実装 |
|---|---|---|---|
| `1` | 標準の iSAM2 による自動補正 | INFO を出して無視 | `glim` の `GlobalMapping` |
| `2` | 地図・因子・変数を蓄積し、大域補正しない | INFO を出して無視 | `glim` の Dummy 保持処理 |
| `3` | 通常は蓄積のみ | INFO を出して要求を登録し、処理境界で全グラフ LM | `glim` の `optimize_manual()` |

`enable_global_mapping=false` または Global Mapping が生成されていない場合、Empty は無視され、その理由を INFO に出します。`enable_optimization=false` は `glim` 側で実効モード `2` になります。設定ファイルに `3` と書いていても、その場合は Empty を受け付けません。

3 モードの実装は `libglobal_mapping.so` が対象です。`libglobal_mapping_pose_graph.so` は現在モード `1` のみ対応し、`2`・`3` を指定すると例外で拒否します。配布されている PoseGraph 設定に切り替える場合も同じ制約があります。

### 4.2 受信から完了まで

```text
Empty
  → GlimROS::optimize_callback()
  → 実効モードを確認し、受信・受付/無視を INFO
  → AsyncGlobalMapping::request_optimization() の atomic フラグ
  → Global Mapping が取り出した現在の入力バッチを処理
  → モード 3 の要求を exchange(false) で取得
  → 全グラフ LM（収束、または最大反復数まで）
  → 全サブマップ姿勢へ反映、更新コールバック
  → 成功した場合だけ完了 INFO
```

後半のスレッド処理と最適化は `glim` 側です。`frontier_glim` が ROS コールバック内で LM を実行する構造ではありません。

- 要求は「1 サブマップごと」ではなく、取り出した **入力バッチ全体の終了後**に確認します。新しい入力の到着や、入力キューが空になることを待つ条件にはなっていません。
- 開始までに届いた複数要求は bool フラグで 1 件に集約します。補正開始後に届いた要求は次回の処理境界用に残ります。
- 対象はその境界までに Global Mapping へ追加済みの全グラフです。補正中に外側のキューへ到着したサブマップは今回の対象に含みません。
- IMU の端点姿勢・速度・bias と拘束も `glim` 側の全因子・全 Values に含めます。
- 空地図や例外では完了 INFO を出しません。Empty の受信 INFO は、最適化の成功を示すログではありません。
- 標準 Viewer の Optimize ボタンからの要求はモード `1` のみに限定され、モード `3` の Empty 要求を代行しません。

送信例:

```bash
ros2 topic pub --once /glim_ros/optimize std_msgs/msg/Empty '{}'
```

## 5. 全体地図 `/glim_ros/map` の定期配信

### 5.1 master と現在の違い

| 項目 | 比較基準の master | 現在 |
|---|---|---|
| 配信の起点 | サブマップ更新コールバックに入ったとき | 入力イベントと独立した Viewer ワーカー |
| 間隔 | コールバック内で固定 10 秒の条件判定 | `glim_ros.ros_pub_interval_time` 秒、既定 `20.0` |
| 時間源 | 独立して生成した ROS clock | 周期は steady clock、stamp は ROS ノードの clock |
| 購読者ゼロ | 配信をスキップ | 購読者数を確認せず合成・publish |
| 入力・更新停止中 | 配信を起こす更新通知が必要 | 保持している最新地図を再配信 |
| 合成 | 配信のたびに再合成 | 点群または姿勢が変わった場合のみ再合成 |
| 点群の保持 | サイズを見て最新サブマップを追記 | 全サブマップの点群・姿勢を対で置換 |

Publisher は `~/map`、型は `sensor_msgs/msg/PointCloud2` です。通常 launch のノード名 `glim_ros` では `/glim_ros/map` になります。QoS は reliable・transient local・keep last 1 で、遅れて購読を始めた側にも最後の地図を保持する設定です。

### 5.2 スレッド間の受け渡しとキャッシュ

1. `GlobalMappingCallbacks::on_update_submaps` で、完成済みの **全サブマップ**の点群共有ポインタと `T_world_origin` を同じ順序で取得します。
2. 配信ワーカー向けの `invoke_queue` に、そのスナップショットを渡します。キューの受け渡しは mutex で保護します。
3. ワーカーの `spin_once()` がキューを取り出します。配列サイズ、点群ポインタ、姿勢行列が前回と同一なら何も変更しません。違えば全配列を置換し、`globalmap_dirty=true` にします。
4. `publish_globalmap()` が周期に達したら、dirty またはキャッシュ未生成の場合だけ各点を対応する姿勢で変換・結合し、シリアライズ済み PointCloud2 をキャッシュします。
5. 変更がない次回以降はキャッシュを再利用し、stamp だけを `node.get_clock()->now()` で更新して配信します。

合成スレッドが、補正中の可変な `SubMap::T_world_origin` を読み続ける構造ではありません。点群は Global Mapping 追加後には変更されない前提で共有します。サブマップ点群は既に Sub Mapping 側で間引かれたものです。全体地図配信側では追加の間引き・同一点の重複除去は行いません。結合するのは座標であり、元点群の intensity や点時刻等はこの結合キャッシュに引き継ぎません。

### 5.3 時間と更新に関する動作

- `ros_pub_interval_time` は **秒数**です。`20.0` は約 20 秒間隔であり、20 Hz ではありません。0 以下・非有限値は例外で拒否します。
- ワーカーは約 10 ms ごとにキューと配信期限を確認します。厳密なリアルタイム周期ではありません。
- 配信終了後の steady 時刻から次の間隔を計ります。合成に時間がかかる場合、publish の開始時刻どうしの間隔にはその処理時間も加わります。遅れを取り戻す連続配信はしません。
- 最初の期限は Publisher 初期化時から計ります。その時点で完成サブマップがなければ配信しません。最初のサブマップ到着が期限より遅ければ、取り込み後のワーカーループで配信可能になります。
- 入力停止後も配信します。`use_sim_time=true` で `/clock` が停止した場合も wall time の周期は進むため、同じ ROS stamp の地図が繰り返し配信される場合があります。
- 補正通知はスナップショットを更新するだけで、補正直後の臨時配信はありません。補正モード `1`・`2`・`3` で配信スケジュールは同じです。
- **合成・publish 中に補正が完了した場合、その処理は旧スナップショットで配信し得ます。** 補正通知は次の `spin_once()` で取り込むので、補正完了後の最初のメッセージに必ず新姿勢が含まれる保証はありません。取り込んだ後の次周期には再合成対象になります。
- 地図が不変であることの判定は、点群ポインタの同一性と姿勢行列の厳密一致です。数値がわずかでも変われば再合成します。点群を同じポインタのまま外部から書き換える利用は想定していません。

## 6. 独自サブマップ配信を追加してから撤去するまで

以下はコミット内容を確認した経緯です。現在の機能と、過去の一時的な機能を混同しないために記載しています。

| コミット | 当時の変更 |
|---|---|
| `ed21720` | 設定セット、launch、install 設定を追加 |
| `d04d241` | Gazebo 用設定を追加・調整 |
| `72edf6c` | `submap_bridge`、独自 `Submap.msg`、生成・依存設定、読み込み設定、RViz 表示を追加。`/glim/submap` にサブマップ情報、`/glim/submaps` に累積点群を配信 |
| `ed6cf5f`・`52ed94f`・`716a048` | 上記モジュールのリンク・C 言語有効化・ROS メッセージ生成を調整 |
| `92f50a4` | 累積点群保持を廃止し、最新単体点群を `/glim/submap_cloud` へ配信する方式に変更 |
| `9985a68` | Empty 購読と手動補正の設定を追加 |
| `cf0ba8e` | bridge・独自メッセージと関連設定を撤去し、標準 `~/map` の定期配信へ変更 |
| 現在の staged 変更 | ROS/Viewer コールバックの寿命管理、終了順序、同一スナップショット時のキャッシュ保持を追加修正 |

現在は、bridge のヘッダ・実装、`msg/Submap.msg`、rosidl 生成・typesupport の設定、独自インターフェース生成依存、拡張モジュール読み込み、RViz の独自表示がありません。旧 `/glim/submap`、`/glim/submaps`、`/glim/submap_cloud` は現在のソースから配信しません。

`std_msgs` は Empty 購読に必要なので残しています。内部の `SubMapping` と `SubMap` は撤去していません。Odometry で周辺化されたフレームを Sub Mapping に渡し、完成サブマップを Global Mapping に追加する流れは継続しています。

## 7. 残っている標準出力と終了処理

### 7.1 ROS の標準出力

独自 bridge を撤去した後も、[RvizViewer](src/glim_ros/rviz_viewer.cpp) の次の系統は残っています。`~` は通常 launch では `/glim_ros` です。

| 出力 | 型・内容 |
|---|---|
| `~/points`、`~/points_corrected` | PointCloud2 |
| `~/aligned_points`、`~/aligned_points_corrected` | PointCloud2 |
| `~/odom`、`~/odom_scanend`、各 `_corrected` | Odometry |
| `~/lidar_odom`、`~/lidar_odom_scanend`、各 `_corrected` | LiDAR 系の Odometry |
| `~/pose`、`~/pose_scanend`、各 `_corrected` | PoseStamped |
| `~/lidar_pose`、`~/lidar_pose_scanend`、各 `_corrected` | LiDAR 系の PoseStamped |
| TF | `map → odom → base_frame`、設定に応じて IMU–LiDAR 間 |

これら通常出力の多くには従来通り購読者数の確認があります。「購読者ゼロでも配信処理する」という変更の対象は全体地図 `~/map` です。master にある共分散・bias 系の追加出力については、次節の未取り込み差分を参照してください。

### 7.2 終了・アンロード時の変更

現在の staged 変更では、GlimROS の ROS コールバックが生の `this` を直接保持する代わりに、共有 dispatcher と `CallbackConnections` を使います。終了時に connection を解除し、進行中の呼び出しが戻るまで待ちます。後から実行される ROS クロージャは dispatcher を持ちますが、解除済みならノードへ入らない構造です。

終了順序は、ROS コールバックの無効化・待機、timer/購読の停止、Odometry → Sub Mapping → Global Mapping の join、必要なら `/tmp/dump` への保存、拡張モジュール破棄です。Global Mapping の join は Local Mapping の有無に依存しない位置へ移しています。`dump_on_unload=false` でも終了時の join は行います。

RvizViewer も登録 connection を解除して進行中コールバックを待ってから配信ワーカーを停止・join します。ワーカー自体は Publisher と ROS clock の初期化後に起動するため、コンストラクタ直後から未初期化 Publisher へアクセスする経路を避けています。

この管理には `glim` 側で追加した `CallbackConnections` が必要です。標準 master の `glim` とこの ROS 側だけを組み合わせて使える、という互換性はありません。

## 8. master 側の更新で現在に含まれていないもの

以下は `HEAD..master` の 7 コミットです。これらを「独自機能のため意図的に削除した」とは解釈できません。分岐元が古いため、現在のソースと master の直接比較に現れる差です。

| master 側のコミット | 現在の差分・影響する場面 |
|---|---|
| `fc72f46` — v1.2.2 | 現在の CMake/package.xml は `1.2.1`。master のバージョン表記は `1.2.2` |
| `4a9e7a4` — auto export | 現在の `offline_viewer` は `--export_path` を受け付けず、master の自動 PLY export 経路を持たない |
| `179bea7` — fix crash on exit | `glim_rosbag` の signal/atexit 経由の端末復元、`rclcpp::ok()` 付き待機条件、明示的な `glim.reset()` 等は現在にない。通常デストラクタでの端末復元はあるが、割り込み・異常終了時の処理は同一ではない |
| `45965b9` — Pub covs | `pose_corrected_with_cov`、`pose_scanend_corrected_with_cov`、LiDAR の対応する共分散トピック、`imu_bias`、`imu_bias_with_cov` は現在にない。Odometry に計算済み共分散を詰める追加処理もない |
| `4dccdaa` — add ament_index_cpp depend | 現在の package.xml に `ament_index_cpp` の明示依存がない。一方、コードは引き続き同ライブラリのヘッダ・API を使う。また、このコミットに含まれる CMake 最小バージョン `3.22` と Boost 用 `CMP0167` 対応も現在にない（現在は `3.16`）。依存解決・ビルドの確認事項 |
| `1093852` — angular velocity scale | 現在は `ang_scale` を読み込まず、IMU 角速度をそのまま利用する。master の角速度スケール設定は使えない |
| `4d4ec52` — duplicate submap fix | コミット自体は未取り込み。ただし現在は全スナップショット置換方式なので、同じ通知のたびに最新サブマップを追記する経路はない |

現在追加した終了管理は ROS ノードと Viewer の寿命を扱うものです。`179bea7` の rosbag CLI 終了処理がすべて取り込まれたことを意味しません。また、独自 bridge 撤去で標準 points/TF/Odometry/Pose 系を残したことと、比較対象 master の全機能を保持していることは別です。

## 9. 確認できた範囲と実行検証

本書は、Git の分岐・コミット内容・現在の staged 変更・呼び出し経路・ワーカー間のキューを静的に確認した説明です。ビルド、QoS の実接続、bag 再生、IMU を含む最適化結果、終了時の競合は未検証です。

実行環境で最低限確認する場合は、次が成功条件になります。

1. **通常起動:** Gazebo launch で指定した入力を購読し、必要な TF がそろい、RViz の `/glim_ros/map` に地図が出る。
2. **モード切替:** 同じ入力で、1 は自動補正、2 は Empty を含め大域補正なし、3 は Empty 前は蓄積のみとなる。3 で補正後に追加し、2 回目の Empty でも全体を補正できる。
3. **要求とログ:** 空地図は完了ログなし。待機中の複数要求は集約され、補正中の要求は次回に残る。成功 INFO は全姿勢への反映後に出る。
4. **定期配信:** 入力停止後も複数周期にわたり配信される。購読者ゼロでも publish 経路が実行され、地図不変時は合成を再実行しない。これは受信側の `echo` だけでは判定できないため、必要ならデバッガ等で確認する。
5. **補正と配信の重なり:** 合成中の補正では旧スナップショットが 1 回配信され得る現仕様を踏まえ、その次の取り込み・周期で補正結果が反映されることを確認する。
6. **終了:** 入力処理中・補正中のアンロードと、`dump_on_unload` の両設定で、解放後アクセス・ハングがないことを確認する。rosbag CLI の割り込み終了は別途確認する。
