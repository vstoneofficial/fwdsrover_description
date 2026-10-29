# 4WDSローバーの3Dモデルのパッケージ

<p align="center">
  <img src="./images/fwdsrover_description.png" width="500" />
</p>

ROS 2 Jazzy / Gazebo Harmonic向けの、`x40a`、`x120a`、`x120a_lb`のモデルです。
3機種とも前方が+X、左が+Y、上が+Zです。`base_footprint`と`base_link`の向きは一致します。

## 準備

Ubuntu 24.04にROS 2 Jazzyをインストールしてから、以下を実行してください。
`src`フォルダーを作る必要はありません。

```bash
source "$(dpkg-query -L ros-jazzy-ros-workspace | awk '/\/setup\.bash$/ {print; exit}')"
mkdir -p fwdsrover_ws
cd fwdsrover_ws
git clone --branch jazzy https://github.com/vstoneofficial/fwdsrover_description.git fwdsrover_description
git clone --branch jazzy https://github.com/vstoneofficial/vs_rover_options_description.git vs_rover_options_description
rosdep install --from-paths fwdsrover_description vs_rover_options_description --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

次回からは、作成したワークスペースのルートへ移動し、起動前に`source install/setup.bash`を実行してください。

1. 対応するオプション
   - LRF TG30（3機種の前後）
   - バンパー、カメラステー、RealSense D435i（`x40a`のみ）

2. 使用する機種のXacro冒頭にある`enable_tg30`などを`true`または`false`に変更してください。
   - [x40a.xacro](./urdf/x40a.xacro)
   - [x120a.xacro](./urdf/x120a.xacro)
   - [x120a_lb.xacro](./urdf/x120a_lb.xacro)

D435iを有効にするとカメラステーも付きます。`x120a`／`x120a_lb`用のバンパー・カメラステーは未提供です。
バンパーは形状と可動関節のモデルで、接触検出トピックは含みません。

## 構成

- `urdf/`：各台車のXacro、サスペンション・アクチュエータの設定
- `meshes/`：3Dモデル
- `launch/`：RViz表示とGazebo起動
- `rviz/`：RVizの表示設定
- `params/`：走行指令を各車輪へ伝える補助ノード
- `worlds/`：Gazeboの床面・照明

## RViz上の可視化

`rover:=`で機種を切り替えます。省略すると`x40a`を表示します。

```bash
ros2 launch fwdsrover_description display.launch.py rover:=x40a
```

```bash
ros2 launch fwdsrover_description display.launch.py rover:=x120a
```

```bash
ros2 launch fwdsrover_description display.launch.py rover:=x120a_lb
```

## Gazeboでのシミュレーション

同じように`rover:=`で機種を選べます。

```bash
ros2 launch fwdsrover_description gazebo.launch.py rover:=x40a
```

別のターミナルでワークスペースを読み込み、走行指令を送ります。

```bash
source install/setup.bash
ros2 topic pub --once /rover_twist geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```

停止するときは、速度を0にします。

```bash
ros2 topic pub --once /rover_twist geometry_msgs/msg/Twist '{}'
```

台車の設定はURDF/Xacroを直接編集してください。外部の物理パラメータ上書きは使用しません。
