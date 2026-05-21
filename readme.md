# honrobo_2026_pkg

本パッケージは、2026年のロボット競技会（本ロボ）向けに開発されたROS 2パッケージです。
センサー統合、CAN通信、PS4コントローラーによる操作、およびWeb socetを介した自動運転機能を備えています。

## 1. セットアップとビルド

まず、ROS 2の環境でパッケージをビルドします。

```bash
cd ~/honrobo_2026
colcon build 
source install/setup.bash
```

## 2. 実行準備 (CAN通信の設定)

`can_node` を使用する前に、SocketCANインターフェース（can0）を起動する必要があります。
USB-CANアダプターを接続し、以下のコマンドを実行してください。
canモジュールのポートを調べるコマンド(`ls /dev/ttyACM*`)を毎回実行すること。抜き差しするとポート番号が変わる可能性あり。

```bash
sudo modprobe slcan
sudo modprobe can
sudo modprobe can_raw
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
```

# canを見るには
candump can0
一行ずつコマンドを実行すること。

## 3. プログラム起動手順

各ノードの役割と起動手順は以下の通りです。複数の端末を立ち上げて、それぞれのノードを起動してください。

### ① センサー統合ノード (`all_senser`)
IMU（ジャイロ）とArduino（OTOS）からデータを取得し、自己位置推定（Odometry）を行います。

*   **機能:** `/odom` トピックに位置情報を配信。シリアルポートの自動検出と権限付与（chmod）を自動で行います。
*   **起動コマンド:**
    ```bash
    ros2 run honrobo_2026_pkg all_senser
    ```

### ② CAN通信ノード (`can_node`)
ROS 2トピックとCANバスの橋渡しを行います。

*   **機能:** `can0` を介してデータを送受信し、現在の通信状況をリアルタイムで画面に表示します。
*   **起動コマンド:**
    ```bash
    ros2 run honrobo_2026_pkg can_node
    ```

### ③ PS4コントローラーノード (`ps4_node`)
接続されたPS4コントローラーの入力を取得します。

*   **機能:** `/ps4_joy` トピックに入力状態を配信。
*   **事前準備:** コントローラーをUSBまたはBluetoothでPCに接続しておいてください。
*   **起動コマンド:**
    ```bash
    ros2 run honrobo_2026_pkg ps4_node
    ```

### ④ ロボット制御メインノード (`roboware_node`)
コントローラー入力や自動運転指令を解釈し、ロボットへのCAN指令を生成します。

*   **機能:** 操作モード（マニュアル/自動）の管理、CAN送信（ID: 0x160等）の実行。
*   **起動コマンド:**
    ```bash
    ros2 run honrobo_2026_pkg roboware_node
    ```

### ⑤ Webナビゲーションノード (`web_nav_node`)
スマホやPCのブラウザからロボットを操作・監視するためのインターフェースを提供します。

*   **機能:** HTTPサーバー(ポート8080)とWebSocketサーバー(ポート8765)を起動。自動運転指令を配信。
*   **IPアドレス確認方法:** 
    ```bash
    hostname -I
    ```
    コマンドを実行して表示されるIPアドレスを使用します。
*   **起動コマンド:**
    ```bash
    ros2 run honrobo_2026_pkg web_nav_node
    ```

## 4. Web UIによる自動運転の使い方

1.  上記すべてのノード（または少なくとも `all_senser`, `can_node`, `roboware_node`, `web_nav_node`）を起動します。
2.  スマホや操作用PCをロボットと同じネットワークに接続します。
3.  ブラウザで以下のURLにアクセスします：
    `http://<ロボットPCのIPアドレス>:8080`
4.  画面上の「TARGET POSITION」に目標座標（X, Y, Z）を入力し、「GO TO TARGETを押すと自動走行が開始されます。
5.  緊急時は「EMERGENCY STOP」ボタン、またはPS4コントローラーの操作（マニュアル復帰）で停止してください。

### ⑥ カメラ配信ノード (`camera_node`)
USBカメラの映像をリアルタイムでスマホやブラウザに配信します。

*   **機能:** GStreamerを使用して低遅延なRTSP配信を開始します。
*   **起動コマンド:**
    ```bash
    ros2 run honrobo_2026_pkg camera_node
    ```

## 4. カメラ配信のセットアップと視聴方法

### 1. Ubuntuに MediaMTX を入れる（初回のみ）
```bash
# 作業用フォルダに移動
cd ~/Downloads

# AMD64版（Inspiron用）をダウンロード
wget https://github.com/bluenviron/mediamtx/releases/download/v1.9.0/mediamtx_v1.9.0_linux_amd64.tar.gz

# 解凍
mkdir -p mediamtx_amd64
tar -xvzf mediamtx_v1.9.0_linux_amd64.tar.gz -C mediamtx_amd64
```

### 2. 配信サーバー（MediaMTX）の起動
```bash
cd ~/Downloads/mediamtx_amd64
./mediamtx
```

### 3. カメラ配信ノードの起動
別のターミナルで、ROS 2 のカメラノードを起動します。
```bash
ros2 run honrobo_2026_pkg camera_node
```

### 4. スマホで映像を見る
スマホのブラウザ（Chromeなど）を開き、以下のURLを入力してください。
```
http://[UbuntuのIPアドレス]:8889/mystream
```

この「8889番ポート」は WebRTC という技術を使っており、YouTubeなどの配信と同じ仕組みで、かつ超低遅延で映像が見れます。

## 4. Nav2/RViz での動作確認手順

###(ここからは不確定要素が多く含まれる内容です。実行する際はほかの方法を行うことをお勧めします。)

プログラム本体（`all_senser.py`など）を書き換えずに、Nav2の動作条件（座標変換）を満たしてRVizで視覚化する手順です。

### ① 座標変換 (TF) のブリッジ
Nav2やRVizで位置を表示するには、`map -> odom -> base_link` という座標の繋がり（TFツリー）が必要です。

1. **map -> odom の固定 (別ターミナル)**
   ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
   ```

2. **odom -> base_link の中継 (別ターミナル)**
   以下の内容を `odom_bridge.py` として保存し、実行します。
   ```python
   import rclpy
   from rclpy.node import Node
   from nav_msgs.msg import Odometry
   from tf2_ros import TransformBroadcaster
   from geometry_msgs.msg import TransformStamped

   class OdomBridge(Node):
       def __init__(self):
           super().__init__('odom_bridge')
           self.br = TransformBroadcaster(self)
           self.create_subscription(Odometry, 'odom', self.callback, 10)

       def callback(self, msg):
           t = TransformStamped()
           t.header = msg.header
           t.child_frame_id = msg.child_frame_id
           t.transform.translation.x = msg.pose.pose.position.x
           t.transform.translation.y = msg.pose.pose.position.y
           t.transform.translation.z = msg.pose.pose.position.z
           t.transform.rotation = msg.pose.pose.orientation
           self.br.sendTransform(t)

   def main():
       rclpy.init()
       rclpy.spin(OdomBridge())
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
   実行コマンド:
   ```bash
   python3 odom_bridge.py
   ```

### ② RViz2 での視覚化
1. **RViz2 を起動**
   ```bash
   rviz2
   ```
2. **基本設定**
   - 左パネルの **Global Options** -> **Fixed Frame** を `map` または `odom` に設定。
3. **項目の追加**
   - 左下の **[Add]** ボタン -> **Odometry** を追加。
   - 追加した Odometry の項目を開き、**Topic** に `/odom` を指定。
   - 必要に応じて **Keep** を 100 程度に増やすと、移動の軌跡が表示されます。
   - **Position Tolerance** を 0.01 などに下げると、細かい移動も描写されます。

