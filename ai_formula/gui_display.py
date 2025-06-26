import sys
import math
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar, QFrame, QSizePolicy
from PySide6.QtCore import QTimer, Qt, Signal, QObject
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QFont
from evdev import InputDevice, categorize, ecodes
import threading
import select
import glob
import os


class SteeringWidget(QWidget):
    """ステアリング表示用の円形ウィジェット"""
    def __init__(self):
        super().__init__()
        self.steering_angle = 0.0  # -1.0 ~ 1.0
        self.setMinimumSize(200, 200)
        
    def set_steering(self, angle):
        """ステアリング角度を設定 (-1.0 ~ 1.0)"""
        self.steering_angle = max(-1.0, min(1.0, angle))
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 背景円を描画
        center_x = self.width() // 2
        center_y = self.height() // 2
        radius = min(center_x, center_y) - 10
        
        # 背景円
        painter.setPen(QPen(QColor(100, 100, 100), 3))
        painter.setBrush(QBrush(QColor(50, 50, 50)))
        painter.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)
        
        # センター線
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        painter.drawLine(center_x, center_y - radius, center_x, center_y + radius)
        painter.drawLine(center_x - radius, center_y, center_x + radius, center_y)
        
        # ステアリング位置を示すライン
        angle_rad = self.steering_angle * math.pi / 2  # -π/2 ~ π/2
        end_x = center_x + int(radius * 0.8 * math.sin(angle_rad))
        end_y = center_y - int(radius * 0.8 * math.cos(angle_rad))
        
        painter.setPen(QPen(QColor(255, 100, 100), 5))
        painter.drawLine(center_x, center_y, end_x, end_y)
        
        # ステアリング値を表示
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 12))
        text = f"{self.steering_angle:.3f}"
        painter.drawText(center_x - 30, center_y + radius + 20, text)


class PedalWidget(QWidget):
    """ペダル表示用のウィジェット"""
    def __init__(self, label_text, color):
        super().__init__()
        self.value = 0.0  # 0.0 ~ 1.0
        self.label_text = label_text
        self.color = color
        self.setMinimumSize(100, 300)
        
    def set_value(self, value):
        """ペダルの値を設定 (0.0 ~ 1.0)"""
        self.value = max(0.0, min(1.0, value))
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # ペダルの背景
        padding = 10
        bar_width = self.width() - 2 * padding
        bar_height = self.height() - 60
        bar_x = padding
        bar_y = 30
        
        # 背景バー
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.setBrush(QBrush(QColor(50, 50, 50)))
        painter.drawRect(bar_x, bar_y, bar_width, bar_height)
        
        # 値バー
        fill_height = int(bar_height * self.value)
        fill_y = bar_y + bar_height - fill_height
        painter.setBrush(QBrush(self.color))
        painter.drawRect(bar_x, fill_y, bar_width, fill_height)
        
        # ラベルと値を表示
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        painter.drawText(bar_x, 20, self.label_text)
        
        painter.setFont(QFont("Arial", 10))
        text = f"{self.value:.3f}"
        painter.drawText(bar_x, self.height() - 5, text)


class InputSignals(QObject):
    """入力データを送信するためのシグナル"""
    steering_changed = Signal(float)
    throttle_changed = Signal(float)
    brake_changed = Signal(float)
    reverse_changed = Signal(float)  # リバースペダル用シグナルを追加
    raw_data_changed = Signal(str)


class HandleInputDisplay(QMainWindow):
    """メインウィンドウクラス"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("G923 ハンドル入力表示")
        self.setGeometry(100, 100, 800, 600)
        
        # シグナルの設定
        self.signals = InputSignals()
        self.signals.steering_changed.connect(self.update_steering)
        self.signals.throttle_changed.connect(self.update_throttle)
        self.signals.brake_changed.connect(self.update_brake)
        self.signals.reverse_changed.connect(self.update_reverse)  # リバース用シグナル接続を追加
        self.signals.raw_data_changed.connect(self.update_raw_data)
        
        self.setup_ui()
        self.setup_input_thread()
        
    def setup_ui(self):
        """UIの設定"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        
        # タイトル
        title_label = QLabel("G923 Racing Wheel Input Monitor")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setStyleSheet("color: white; background-color: #2b2b2b; padding: 10px; border-radius: 5px;")
        main_layout.addWidget(title_label)
        
        # メインコンテンツ
        content_layout = QHBoxLayout()
        
        # ステアリング表示
        steering_frame = QFrame()
        steering_frame.setFrameStyle(QFrame.Box)
        steering_frame.setStyleSheet("background-color: #3b3b3b; border-radius: 10px;")
        steering_layout = QVBoxLayout(steering_frame)
        
        steering_label = QLabel("Steering")
        steering_label.setAlignment(Qt.AlignCenter)
        steering_label.setFont(QFont("Arial", 14, QFont.Bold))
        steering_label.setStyleSheet("color: white; padding: 5px;")
        steering_layout.addWidget(steering_label)
        
        self.steering_widget = SteeringWidget()
        steering_layout.addWidget(self.steering_widget)
        content_layout.addWidget(steering_frame)
        
        # ペダル表示
        pedals_frame = QFrame()
        pedals_frame.setFrameStyle(QFrame.Box)
        pedals_frame.setStyleSheet("background-color: #3b3b3b; border-radius: 10px;")
        pedals_layout = QHBoxLayout(pedals_frame)
        
        self.throttle_widget = PedalWidget("Throttle", QColor(100, 255, 100))
        self.brake_widget = PedalWidget("Brake", QColor(255, 100, 100))
        self.reverse_widget = PedalWidget("Reverse", QColor(100, 100, 255))  # リバースペダル追加
        
        pedals_layout.addWidget(self.throttle_widget)
        pedals_layout.addWidget(self.brake_widget)
        pedals_layout.addWidget(self.reverse_widget)  # リバースペダルをレイアウトに追加
        content_layout.addWidget(pedals_frame)
        
        main_layout.addLayout(content_layout)
        
        # 生データ表示
        raw_data_frame = QFrame()
        raw_data_frame.setFrameStyle(QFrame.Box)
        raw_data_frame.setStyleSheet("background-color: #3b3b3b; border-radius: 10px;")
        raw_data_frame.setFixedHeight(150)  # 高さを固定
        raw_data_layout = QVBoxLayout(raw_data_frame)
        
        raw_data_label = QLabel("Raw Input Data")
        raw_data_label.setFont(QFont("Arial", 12, QFont.Bold))
        raw_data_label.setStyleSheet("color: white; padding: 5px;")
        raw_data_layout.addWidget(raw_data_label)
        
        self.raw_data_display = QLabel("Waiting for input...")
        self.raw_data_display.setFont(QFont("Courier", 10))
        self.raw_data_display.setStyleSheet("color: #cccccc; padding: 10px; background-color: #2b2b2b; border-radius: 5px;")
        self.raw_data_display.setAlignment(Qt.AlignTop)
        self.raw_data_display.setWordWrap(True)
        self.raw_data_display.setFixedHeight(100)  # 高さを固定
        # サイズポリシーを正しく設定
        self.raw_data_display.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        raw_data_layout.addWidget(self.raw_data_display)
        
        main_layout.addWidget(raw_data_frame)
        
        # ウィンドウサイズを固定
        self.setFixedSize(800, 600)
        
        # 全体のスタイル設定
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
            QWidget {
                color: white;
                background-color: #2b2b2b;
            }
        """)
        
    def find_g923_device(self):
        """G923/G29ハンドルデバイスを自動検出"""
        print("デバイス検索を開始します...")
        
        # 方法1: G29の固定的なby-idパスを使用
        g29_stable_path = '/dev/input/by-id/usb-Logitech_G29_Driving_Force_Racing_Wheel-event-joystick'
        if os.path.exists(g29_stable_path):
            try:
                device = InputDevice(g29_stable_path)
                print(f'Found G29 at stable path: {g29_stable_path}')
                return device
            except Exception as e:
                print(f'Failed to open G29 stable path {g29_stable_path}: {e}')
        
        # 方法2: G923の固定的なby-idパスを使用
        g923_stable_path = '/dev/input/by-id/usb-Logitech_G923_Racing_Wheel_for_PlayStation_4_and_PC_USYMUGUXEREJOFORUFUMEZIDU-event-joystick'
        if os.path.exists(g923_stable_path):
            try:
                device = InputDevice(g923_stable_path)
                print(f'Found G923 at stable path: {g923_stable_path}')
                return device
            except Exception as e:
                print(f'Failed to open G923 stable path {g923_stable_path}: {e}')
        
        # 方法3: by-idディレクトリ内でG923/G29を検索
        by_id_patterns = [
            '/dev/input/by-id/*G29*event-joystick',
            '/dev/input/by-id/*G923*event-joystick',
            '/dev/input/by-id/*Logitech*event-joystick'
        ]
        for pattern in by_id_patterns:
            matching_devices = glob.glob(pattern)
            print(f"Pattern {pattern} matched: {matching_devices}")
            for device_path in matching_devices:
                try:
                    device = InputDevice(device_path)
                    print(f'Found Logitech wheel at: {device_path}')
                    return device
                except Exception as e:
                    print(f'Failed to open {device_path}: {e}')
        
        # 方法4: 全デバイスを検索してG923/G29を見つける
        device_pattern = '/dev/input/event*'
        print("すべてのeventデバイスを検索中...")
        for device_path in glob.glob(device_pattern):
            try:
                device = InputDevice(device_path)
                device_name = device.name.lower()
                print(f"チェック中: {device_path} - {device.name}")
                if (('g923' in device_name and 'racing wheel' in device_name) or 
                    ('g29' in device_name and ('racing wheel' in device_name or 'driving force' in device_name)) or
                    ('logitech' in device_name and ('racing wheel' in device_name or 'driving force' in device_name))):
                    print(f'Found Logitech Racing Wheel at: {device_path} ({device.name})')
                    return device
                device.close()
            except Exception as e:
                print(f"Error checking {device_path}: {e}")
                continue
        
        print("G923/G29デバイスが見つかりませんでした")
        return None
        
    def setup_input_thread(self):
        """入力処理スレッドの設定"""
        self.g923 = self.find_g923_device()
        if self.g923 is None:
            self.raw_data_display.setText("G923 Racing Wheel not found!")
            return
            
        self.raw_data_display.setText(f"Connected to: {self.g923.name}")
        
        self.running = True
        self.input_thread = threading.Thread(target=self.handle_input, daemon=True)
        self.input_thread.start()
        
    def handle_input(self):
        """バックグラウンドスレッドでハンドル入力を処理"""
        try:
            while self.running:
                r, w, x = select.select([self.g923.fd], [], [], 0.01)
                if r:
                    try:
                        events = self.g923.read()
                        for event in events:
                            if event.type == ecodes.EV_ABS:
                                abs_event = categorize(event)
                                raw_val = abs_event.event.value
                                
                                if abs_event.event.code == ecodes.ABS_X:
                                    # ステアリング: -1.0 ～ 1.0に正規化
                                    steering_normalized = (raw_val - 32768) / 32768.0
                                    sign = 1 if steering_normalized >= 0 else -1
                                    abs_steering = abs(steering_normalized)
                                    exponential_steering = sign * (abs_steering ** 2) * 2.0
                                    
                                    self.signals.steering_changed.emit(-exponential_steering)
                                    self.signals.raw_data_changed.emit(
                                        f"STEERING: raw={raw_val} normalized={steering_normalized:.3f} exponential={exponential_steering:.3f}"
                                    )
                                    
                                elif abs_event.event.code == ecodes.ABS_RZ:
                                    # ブレーキ: 0-1.0に正規化
                                    brake_normalized = raw_val / 255.0
                                    self.signals.brake_changed.emit(brake_normalized)
                                    self.signals.raw_data_changed.emit(
                                        f"BRAKE: raw={raw_val} normalized={brake_normalized:.3f}"
                                    )
                                    
                                elif abs_event.event.code == ecodes.ABS_Z:
                                    # スロットル: 0-1.0に正規化
                                    throttle_normalized = 1.0 - (raw_val / 255.0)
                                    self.signals.throttle_changed.emit(throttle_normalized)
                                    self.signals.raw_data_changed.emit(
                                        f"THROTTLE: raw={raw_val} normalized={throttle_normalized:.3f}"
                                    )
                                    
                                elif abs_event.event.code == ecodes.ABS_Y:
                                    # リバース（Y軸）: 0-1.0に正規化
                                    reverse_normalized = raw_val / 255.0
                                    self.signals.reverse_changed.emit(reverse_normalized)
                                    self.signals.raw_data_changed.emit(
                                        f"REVERSE: raw={raw_val} normalized={reverse_normalized:.3f}"
                                    )
                                    
                    except BlockingIOError:
                        continue
                    except Exception as e:
                        print(f"Error reading input: {e}")
                        
        except Exception as e:
            print(f"Input thread error: {e}")
            
    def update_steering(self, angle):
        """ステアリング表示を更新"""
        self.steering_widget.set_steering(angle)
        
    def update_throttle(self, value):
        """スロットル表示を更新"""
        self.throttle_widget.set_value(value)
        
    def update_brake(self, value):
        """ブレーキ表示を更新"""
        self.brake_widget.set_value(value)
        
    def update_reverse(self, value):
        """リバースペダル表示を更新"""
        self.reverse_widget.set_value(value)
        
    def update_raw_data(self, data):
        """生データ表示を更新"""
        current_text = self.raw_data_display.text()
        lines = current_text.split('\n')
        if len(lines) > 10:  # 最大10行まで表示
            lines = lines[-9:]  # 最新の9行を保持
        lines.append(data)
        self.raw_data_display.setText('\n'.join(lines))
        
    def closeEvent(self, event):
        """アプリケーション終了時のクリーンアップ"""
        self.running = False
        if hasattr(self, 'input_thread'):
            self.input_thread.join(timeout=1.0)
        if hasattr(self, 'g923'):
            self.g923.close()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    # ダークテーマの設定
    app.setStyle('Fusion')
    
    window = HandleInputDisplay()
    window.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()