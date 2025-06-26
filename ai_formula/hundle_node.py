import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from evdev import InputDevice, categorize, ecodes
import threading
import select
import glob
import os

class HundleNode(Node):
    def __init__(self):
        super().__init__('hundle_node')
        self.publisher_ = self.create_publisher(Twist, '/aiformula_control/twist_mux/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # G923ハンドルを自動検出
        self.g923 = self.find_g923_device()
        if self.g923 is None:
            self.get_logger().error('G923 Racing Wheel not found!')
            return
        
        self.get_logger().info(f'Successfully connected to device: {self.g923.name}')
        
        self.linear_gain = 1.0
        self.angular_gain = 9.42 / 255  # 正規化された値（0-1）に対して適用
        
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        
        # 入力状態を管理する変数を追加
        self.throttle_value = 0.0
        self.brake_active = False
        self.reverse_value = 0.0
        
        # バックグラウンドスレッドでハンドル入力を処理
        self.running = True
        self.input_thread = threading.Thread(target=self.handle_input, daemon=True)  
        self.input_thread.start()
    
    def find_g923_device(self):
        """G923/G29ハンドルデバイスを自動検出"""
        # 方法1: 固定的なby-idパスを使用（G923）
        g923_stable_path = '/dev/input/by-id/usb-Logitech_G923_Racing_Wheel_for_PlayStation_4_and_PC_USYMUGUXEREJOFORUFUMEZIDU-event-joystick'
        if os.path.exists(g923_stable_path):
            try:
                device = InputDevice(g923_stable_path)
                self.get_logger().info(f'Found G923 at stable path: {g923_stable_path}')
                return device
            except Exception as e:
                self.get_logger().warn(f'Failed to open G923 stable path {g923_stable_path}: {e}')
        
        # 方法2: by-idディレクトリ内でG923/G29を検索
        by_id_patterns = [
            '/dev/input/by-id/*G923*event-joystick',
            '/dev/input/by-id/*G29*event-joystick'
        ]
        for pattern in by_id_patterns:
            matching_devices = glob.glob(pattern)
            for device_path in matching_devices:
                try:
                    device = InputDevice(device_path)
                    self.get_logger().info(f'Found Logitech wheel at: {device_path}')
                    return device
                except Exception as e:
                    self.get_logger().warn(f'Failed to open {device_path}: {e}')
        
        # 方法3: 全デバイスを検索してG923/G29を見つける
        device_pattern = '/dev/input/event*'
        for device_path in glob.glob(device_pattern):
            try:
                device = InputDevice(device_path)
                device_name = device.name.lower()
                # G923, G29, またはLogitech Racing Wheelを含むデバイスを検索
                if (('g923' in device_name and 'racing wheel' in device_name) or 
                    ('g29' in device_name and 'racing wheel' in device_name) or
                    ('logitech' in device_name and 'racing wheel' in device_name)):
                    self.get_logger().info(f'Found Logitech Racing Wheel at: {device_path} ({device.name})')
                    return device
                device.close()
            except Exception:
                continue
        
        return None
    
    def normalize(self, value, min_value=0, max_value=255):
        return (value - min_value) / (max_value - min_value)
    
    def handle_input(self):
        """バックグラウンドスレッドでハンドル入力を処理"""
        try:
            while self.running:
                # ノンブロッキング読み取り
                r, w, x = select.select([self.g923.fd], [], [], 0.01)
                if r:
                    try:
                        events = self.g923.read()
                        for event in events:
                            if event.type == ecodes.EV_ABS:
                                abs_event = categorize(event)
                                raw_val = abs_event.event.value
                                
                                if abs_event.event.code == ecodes.ABS_X:
                                    # ステアリング: G923/G29の実際の値範囲に基づいて正規化
                                    # G923/G29のステアリング範囲は通常0-65535 (16bit)
                                    steering_normalized = (raw_val - 32768) / 32768.0  # -1.0 ～ 1.0
                                    
                                    # 指数関数的に変化させる（符号を保持）
                                    sign = 1 if steering_normalized >= 0 else -1
                                    abs_steering = abs(steering_normalized)
                                    # 指数関数的変化: y = x^2 で滑らかな変化、最大値2
                                    exponential_steering = sign * (abs_steering ** 2) * 2.0
                                    
                                    self.twist_msg.angular.z = -exponential_steering  # 符号を反転
                                    self.get_logger().info(f"STEERING: raw={raw_val} normalized={steering_normalized:.3f} exponential={exponential_steering:.3f} angular.z={self.twist_msg.angular.z:.3f}")
                                    
                                elif abs_event.event.code == ecodes.ABS_RZ:
                                    # ブレーキ: 0-255の範囲
                                    brake_raw = raw_val
                                    if brake_raw < 240:  # ブレーキが踏まれている
                                        brake_normalized = (240 - brake_raw) / 240.0
                                        self.brake_active = brake_normalized > 0.1
                                        self.get_logger().info(f"BRAKE: raw={raw_val} normalized={brake_normalized:.3f} active={self.brake_active}")
                                    else:  # ブレーキが離されている
                                        self.brake_active = False
                                    
                                    # linear.xの値を更新
                                    self.update_linear_velocity()
                                        
                                elif abs_event.event.code == ecodes.ABS_Z:
                                    # スロットル: 0-255の範囲、逆転が必要
                                    throttle_raw = raw_val
                                    if throttle_raw < 250:
                                        self.throttle_value = (250 - throttle_raw) / 250.0 * self.linear_gain
                                        self.get_logger().info(f"THROTTLE: raw={raw_val} normalized={self.throttle_value/self.linear_gain:.3f} throttle_value={self.throttle_value:.3f}")
                                    else:
                                        self.throttle_value = 0.0
                                    
                                    # linear.xの値を更新
                                    self.update_linear_velocity()
                                    
                                elif abs_event.event.code == ecodes.ABS_Y:
                                    # リバース（Y軸）: 0-255の範囲
                                    reverse_raw = raw_val
                                    if reverse_raw < 240:  # リバースが押されている
                                        reverse_normalized = (240 - reverse_raw) / 240.0
                                        self.reverse_value = -reverse_normalized * self.linear_gain  # 負の値
                                        self.get_logger().info(f"REVERSE: raw={raw_val} normalized={reverse_normalized:.3f} reverse_value={self.reverse_value:.3f}")
                                    else:  # リバースが離されている
                                        self.reverse_value = 0.0
                                    
                                    # linear.xの値を更新
                                    self.update_linear_velocity()
                                    
                    except BlockingIOError:
                        continue
                    except Exception as e:
                        self.get_logger().error(f"Error reading input: {e}")
                        
        except Exception as e:
            self.get_logger().error(f"Input thread error: {e}")
    
    def update_linear_velocity(self):
        """スロットル、ブレーキ、リバースの状態に基づいてlinear.xを更新"""
        if self.brake_active:
            # ブレーキが踏まれている場合は停止
            self.twist_msg.linear.x = 0.0
        elif self.reverse_value != 0.0:
            # リバースが押されている場合は後進
            self.twist_msg.linear.x = self.reverse_value
        else:
            # 通常のスロットル操作
            self.twist_msg.linear.x = self.throttle_value
    
    def timer_callback(self):
        """定期的にTwistメッセージを送信"""
        self.publisher_.publish(self.twist_msg)
        # デバッグ情報を定期的に出力（値が0でない場合のみ）
        if self.twist_msg.linear.x != 0.0 or self.twist_msg.angular.z != 0.0:
            self.get_logger().info(f"PUBLISHED: linear.x={self.twist_msg.linear.x:.3f}, angular.z={self.twist_msg.angular.z:.3f}")
        
        # パブリッシャーの接続状況を確認
        subscriber_count = self.publisher_.get_subscription_count()
        if subscriber_count == 0:
            self.get_logger().warn("No subscribers connected to /aiformula_control/twist_mux/cmd_vel topic!")
        
    def destroy_node(self):
        """ノード終了時のクリーンアップ"""
        self.running = False
        if hasattr(self, 'input_thread'):
            self.input_thread.join(timeout=1.0)
        if hasattr(self, 'g923'):
            self.g923.close()
        super().destroy_node()
            
def main(args=None):
    rclpy.init(args=args)
    hundle_node = HundleNode()
    
    try:
        rclpy.spin(hundle_node)  # ここでROS2のスピナーを開始
    except KeyboardInterrupt:
        hundle_node.get_logger().info("Shutting down...")
    finally:
        hundle_node.destroy_node()
        rclpy.shutdown()
    
        
if __name__ == '__main__':
    main()

