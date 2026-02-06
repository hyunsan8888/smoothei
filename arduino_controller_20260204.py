#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Arduino Mega 2560 산업용 자동화 시스템 제어 프로그램
2단계 UI: 고객용 화면 + 관리자용 제어 화면
"""

import sys
import json
import time
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QLabel, QTextEdit, QGridLayout,
    QStatusBar, QMessageBox, QComboBox, QStackedWidget
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread, QPoint
from PyQt5.QtGui import QFont, QColor, QPalette
import serial
import serial.tools.list_ports


class SerialThread(QThread):
    """시리얼 통신 스레드"""
    status_received = pyqtSignal(dict)
    log_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool, str)
    
    def __init__(self, port='COM4', baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        
    def run(self):
        """스레드 실행"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=1.0
            )
            time.sleep(2)
            self.running = True
            msg = f"✓ {self.port} 연결 성공 ({self.baudrate} bps)"
            self.log_received.emit(msg)
            self.connection_status.emit(True, msg)
            
            buffer = ""
            while self.running:
                if self.serial.in_waiting > 0:
                    try:
                        data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                        buffer += data
                        
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            
                            if line:
                                if line.startswith('{') and '"type":"status"' in line:
                                    try:
                                        status_data = json.loads(line)
                                        self.status_received.emit(status_data)
                                    except json.JSONDecodeError:
                                        self.log_received.emit(line)
                                else:
                                    self.log_received.emit(line)
                    except Exception as e:
                        self.log_received.emit(f"읽기 오류: {str(e)}")
                
                time.sleep(0.01)
                
        except serial.SerialException as e:
            msg = f"✗ 시리얼 포트 오류: {str(e)}"
            self.log_received.emit(msg)
            self.connection_status.emit(False, msg)
        except Exception as e:
            msg = f"✗ 예외 발생: {str(e)}"
            self.log_received.emit(msg)
            self.connection_status.emit(False, msg)
        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.log_received.emit("시리얼 포트 닫힘")
            self.connection_status.emit(False, "연결 끊김")
    
    def send_command(self, command):
        """명령 전송"""
        if self.serial and self.serial.is_open and self.running:
            try:
                self.serial.write(f"{command}\n".encode('utf-8'))
                self.log_received.emit(f"→ 명령 전송: {command}")
                return True
            except Exception as e:
                self.log_received.emit(f"✗ 전송 오류: {str(e)}")
                return False
        return False
    
    def stop(self):
        """스레드 중지"""
        self.running = False
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except:
                pass


class CustomerScreen(QWidget):
    """고객용 화면 (첫 번째 화면)"""
    start_clicked = pyqtSignal()
    stop_clicked = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.photo_detected = False
        self.secret_click_count = 0
        self.last_click_time = 0
        self.initUI()
        
    def initUI(self):
        """UI 초기화"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 메인 컨테이너
        main_container = QWidget()
        main_layout = QVBoxLayout(main_container)
        
        # 상태 메시지 라벨 (큰 텍스트)
        self.message_label = QLabel("스무디를 바코드에\n올려 주세요")
        self.message_label.setAlignment(Qt.AlignCenter)
        self.message_label.setStyleSheet("""
            QLabel {
                font-size: 48px;
                font-weight: bold;
                color: #2196F3;
                padding: 50px;
                background-color: white;
            }
        """)
        main_layout.addWidget(self.message_label)
        
        # 버튼 컨테이너 (처음에는 숨김)
        self.button_container = QWidget()
        button_layout = QHBoxLayout(self.button_container)
        button_layout.setSpacing(30)
        
        # 시작 버튼
        self.start_btn = QPushButton("시작")
        self.start_btn.setMinimumSize(300, 200)
        self.start_btn.setStyleSheet("""
            QPushButton {
                font-size: 48px;
                font-weight: bold;
                background-color: #4CAF50;
                color: white;
                border-radius: 20px;
                border: 5px solid #45a049;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        self.start_btn.clicked.connect(self.on_start_clicked)
        button_layout.addWidget(self.start_btn)
        
        # 종료 버튼
        self.stop_btn = QPushButton("종료")
        self.stop_btn.setMinimumSize(300, 200)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                font-size: 48px;
                font-weight: bold;
                background-color: #f44336;
                color: white;
                border-radius: 20px;
                border: 5px solid #da190b;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:pressed {
                background-color: #c1170a;
            }
        """)
        self.stop_btn.clicked.connect(self.on_stop_clicked)
        button_layout.addWidget(self.stop_btn)
        
        self.button_container.setVisible(False)
        main_layout.addWidget(self.button_container)
        
        layout.addWidget(main_container)
        
        # 숨겨진 관리자 버튼 (오른쪽 상단 모서리)
        self.secret_btn = QPushButton(self)
        self.secret_btn.setGeometry(self.width() - 50, 0, 50, 50)
        self.secret_btn.setStyleSheet("background-color: transparent; border: none;")
        self.secret_btn.clicked.connect(self.on_secret_click)
        
    def resizeEvent(self, event):
        """창 크기 변경 시 숨겨진 버튼 위치 조정"""
        super().resizeEvent(event)
        self.secret_btn.setGeometry(self.width() - 50, 0, 50, 50)
    
    def on_secret_click(self):
        """숨겨진 버튼 클릭 (4번 연속 클릭)"""
        current_time = time.time()
        
        # 2초 이내 클릭이면 카운트 증가
        if current_time - self.last_click_time < 2:
            self.secret_click_count += 1
        else:
            self.secret_click_count = 1
        
        self.last_click_time = current_time
        
        # 4번 클릭하면 관리자 화면으로 전환
        if self.secret_click_count >= 4:
            self.parent().parent().switch_to_advanced()
            self.secret_click_count = 0
    
    def update_photo_sensor(self, detected):
        """포토센서 상태 업데이트"""
        if detected and not self.photo_detected:
            # 감지됨 → 바로 버튼 표시
            self.show_buttons()
        elif not detected and self.photo_detected:
            # 감지 안 됨 → 대기 메시지로 복귀
            self.show_waiting_message()
        
        self.photo_detected = detected
    
    def show_buttons(self):
        """시작/종료 버튼 바로 표시"""
        self.message_label.setVisible(False)
        self.button_container.setVisible(True)
    
    def show_waiting_message(self):
        """대기 메시지 표시"""
        self.message_label.setText("스무디를 바코드에\n올려 주세요")
        self.message_label.setStyleSheet("""
            QLabel {
                font-size: 48px;
                font-weight: bold;
                color: #2196F3;
                padding: 50px;
                background-color: white;
            }
        """)
        self.message_label.setVisible(True)
        self.button_container.setVisible(False)
    
    def on_start_clicked(self):
        """시작 버튼 클릭"""
        self.start_clicked.emit()
    
    def on_stop_clicked(self):
        """정지 버튼 클릭"""
        self.stop_clicked.emit()


class AdvancedControlScreen(QWidget):
    """관리자용 제어 화면 (두 번째 화면)"""
    
    def __init__(self):
        super().__init__()
        self.initUI()
        
    def initUI(self):
        """UI 초기화"""
        main_layout = QHBoxLayout(self)
        
        # 왼쪽 패널
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel, 1)
        
        # 오른쪽 패널
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, 1)
    
    def create_left_panel(self):
        """왼쪽 제어 패널"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 뒤로가기 버튼
        back_btn = QPushButton("← 고객 화면으로")
        back_btn.setStyleSheet("background-color: #9E9E9E; color: white; padding: 10px; font-weight: bold;")
        back_btn.clicked.connect(lambda: self.parent().parent().switch_to_customer())
        layout.addWidget(back_btn)
        
        # 메인 제어
        control_group = self.create_control_group()
        layout.addWidget(control_group)
        
        # 세척 제어 (새로 추가)
        cleaning_group = self.create_cleaning_group()
        layout.addWidget(cleaning_group)
        
        # 시스템 상태
        status_group = self.create_status_group()
        layout.addWidget(status_group)
        
        # 센서 상태
        sensor_group = self.create_sensor_group()
        layout.addWidget(sensor_group)
        
        # 수동 제어
        manual_group = self.create_manual_control_group()
        layout.addWidget(manual_group)
        
        layout.addStretch()
        return panel
    
    def create_control_group(self):
        """메인 제어 그룹"""
        group = QGroupBox("메인 제어")
        layout = QGridLayout()
        
        self.start_btn = QPushButton("시작 (START)")
        self.start_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.start_btn.setMinimumHeight(50)
        layout.addWidget(self.start_btn, 0, 0)
        
        self.stop_btn = QPushButton("정지 (STOP)")
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.stop_btn.setMinimumHeight(50)
        layout.addWidget(self.stop_btn, 0, 1)
        
        self.home_btn = QPushButton("홈 복귀 (HOME)")
        self.home_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        self.home_btn.setMinimumHeight(50)
        layout.addWidget(self.home_btn, 1, 0)
        
        self.status_btn = QPushButton("상태 조회 (STATUS)")
        self.status_btn.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
        self.status_btn.setMinimumHeight(50)
        layout.addWidget(self.status_btn, 1, 1)
        
        group.setLayout(layout)
        return group
    
    def create_cleaning_group(self):
        """세척 제어 그룹"""
        group = QGroupBox("세척 제어")
        layout = QVBoxLayout()
        
        info_label = QLabel("작업 완료 후 컵을 제거하고\n세척을 시작하세요")
        info_label.setStyleSheet("color: #666; font-size: 11px;")
        info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(info_label)
        
        self.cleaning_start_btn = QPushButton("세척 시작")
        self.cleaning_start_btn.setStyleSheet("""
            background-color: #00BCD4; 
            color: white; 
            font-weight: bold; 
            padding: 15px;
            font-size: 14px;
        """)
        self.cleaning_start_btn.setMinimumHeight(60)
        layout.addWidget(self.cleaning_start_btn)
        
        group.setLayout(layout)
        return group
    
    def create_status_group(self):
        """시스템 상태 그룹"""
        group = QGroupBox("시스템 상태")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("상태:"), 0, 0)
        self.state_label = QLabel("알 수 없음")
        self.state_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self.state_label, 0, 1)
        
        layout.addWidget(QLabel("단계:"), 1, 0)
        self.step_label = QLabel("-")
        layout.addWidget(self.step_label, 1, 1)
        
        group.setLayout(layout)
        return group
    
    def create_sensor_group(self):
        """센서 상태 그룹"""
        group = QGroupBox("센서 상태")
        layout = QGridLayout()
        
        self.sensor_labels = {}
        sensors = [
            ('photo', '포토센서'),
            ('dc_top', 'DC 상단'),
            ('dc_bottom', 'DC 하단'),
            ('stp_top', '스테퍼 상단'),
            ('stp_bottom', '스테퍼 하단')
        ]
        
        for idx, (key, name) in enumerate(sensors):
            layout.addWidget(QLabel(f"{name}:"), idx, 0)
            label = QLabel("●")
            label.setStyleSheet("color: gray;")
            self.sensor_labels[key] = label
            layout.addWidget(label, idx, 1)
        
        group.setLayout(layout)
        return group
    
    def create_manual_control_group(self):
        """수동 제어 그룹"""
        group = QGroupBox("수동 제어")
        layout = QGridLayout()
        
        self.manual_buttons = []
        controls = [
            ('VALVE1', '밸브1'),
            ('VALVE2', '밸브2'),
            ('PUMP', '펌프'),
            ('BLENDER', '블렌더'),
            ('HEATER', '히터')
        ]
        
        for idx, (cmd, name) in enumerate(controls):
            on_btn = QPushButton(f"{name} ON")
            on_btn.setObjectName(f"{cmd}:ON")
            layout.addWidget(on_btn, idx, 0)
            
            off_btn = QPushButton(f"{name} OFF")
            off_btn.setObjectName(f"{cmd}:OFF")
            layout.addWidget(off_btn, idx, 1)
            
            self.manual_buttons.append(on_btn)
            self.manual_buttons.append(off_btn)
        
        group.setLayout(layout)
        return group
    
    def create_right_panel(self):
        """오른쪽 로그 패널"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        log_group = QGroupBox("통신 로그")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        log_layout.addWidget(self.log_text)
        
        clear_btn = QPushButton("로그 지우기")
        clear_btn.clicked.connect(self.log_text.clear)
        log_layout.addWidget(clear_btn)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        return panel


class ArduinoControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_thread = None
        self.current_state = {}
        self.is_connected = False
        self.auto_connect_tried = False
        self.initUI()
        
        # 자동 연결 시도
        QTimer.singleShot(500, self.try_auto_connect)
        
    def initUI(self):
        """UI 초기화"""
        self.setWindowTitle('스무디 자동화 시스템')
        self.setGeometry(100, 100, 1200, 800)
        
        # 스택 위젯 (화면 전환용)
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)
        
        # 고객 화면
        self.customer_screen = CustomerScreen()
        self.customer_screen.start_clicked.connect(self.on_customer_start)
        self.customer_screen.stop_clicked.connect(self.on_customer_stop)
        self.stack.addWidget(self.customer_screen)
        
        # 관리자 화면
        self.advanced_screen = AdvancedControlScreen()
        self.setup_advanced_connections()
        self.stack.addWidget(self.advanced_screen)
        
        # 상태바
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('시스템 시작 중...')
        
        # 첫 화면 표시
        self.stack.setCurrentWidget(self.customer_screen)
    
    def setup_advanced_connections(self):
        """관리자 화면 연결 설정"""
        adv = self.advanced_screen
        
        # 메인 제어
        adv.start_btn.clicked.connect(lambda: self.send_command("START"))
        adv.stop_btn.clicked.connect(lambda: self.send_command("STOP"))
        adv.home_btn.clicked.connect(lambda: self.send_command("HOME"))
        adv.status_btn.clicked.connect(lambda: self.send_command("STATUS"))
        
        # 세척 제어
        adv.cleaning_start_btn.clicked.connect(self.on_cleaning_start)
        
        # 수동 제어
        for btn in adv.manual_buttons:
            btn.clicked.connect(lambda checked, b=btn: self.send_command(b.objectName()))
    
    def try_auto_connect(self):
        """자동 연결 시도"""
        if self.auto_connect_tried:
            return
        
        self.auto_connect_tried = True
        ports = serial.tools.list_ports.comports()
        
        if ports:
            # 첫 번째 포트로 자동 연결
            port = ports[0].device
            self.add_log(f"자동 연결 시도: {port}")
            self.connect_serial(port)
        else:
            self.add_log("사용 가능한 포트 없음")
            self.statusBar.showMessage('시리얼 포트 없음 - 수동 연결 필요')
    
    def connect_serial(self, port):
        """시리얼 연결"""
        if self.is_connected:
            return
        
        self.serial_thread = SerialThread(port=port, baudrate=115200)
        self.serial_thread.status_received.connect(self.update_status)
        self.serial_thread.log_received.connect(self.add_log)
        self.serial_thread.connection_status.connect(self.on_connection_status)
        self.serial_thread.start()
    
    def on_connection_status(self, connected, message):
        """연결 상태 변경"""
        self.is_connected = connected
        if connected:
            self.statusBar.showMessage('연결됨 - 작동 준비 완료')
        else:
            self.statusBar.showMessage('연결 끊김')
    
    def switch_to_customer(self):
        """고객 화면으로 전환"""
        self.stack.setCurrentWidget(self.customer_screen)
        self.add_log("고객 화면으로 전환")
    
    def switch_to_advanced(self):
        """관리자 화면으로 전환"""
        self.stack.setCurrentWidget(self.advanced_screen)
        self.add_log("관리자 화면으로 전환")
    
    def on_customer_start(self):
        """고객 화면 시작 버튼"""
        self.send_command("START")
        self.customer_screen.show_waiting_message()  # 버튼 숨기고 대기 메시지
    
    def on_customer_stop(self):
        """고객 화면 정지 버튼"""
        reply = QMessageBox.question(
            self, '확인', 
            '비상 정지하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.send_command("STOP")
    
    def on_cleaning_start(self):
        """세척 시작 버튼"""
        # 세척은 작업 완료 후 컵 제거 상태에서만 가능
        # Arduino에서 WAITING_FOR_REMOVAL 상태일 때 START 버튼 누르면 세척 시작
        current_state = self.current_state.get('state', '')
        
        if 'CLEANING' in current_state or 'WAITING' in current_state:
            reply = QMessageBox.question(
                self, '세척 시작', 
                '세척을 시작하시겠습니까?\n(컵이 제거되었는지 확인하세요)',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.send_command("START")
                self.add_log("세척 프로세스 시작")
        else:
            QMessageBox.warning(self, '경고', '세척은 작업 완료 후에만 가능합니다.')
    
    def send_command(self, command):
        """명령 전송"""
        if not self.is_connected or not self.serial_thread:
            QMessageBox.warning(self, "오류", "시리얼 포트가 연결되어 있지 않습니다")
            return
        
        self.serial_thread.send_command(command)
    
    def update_status(self, status_data):
        """상태 업데이트"""
        self.current_state = status_data
        
        # 포토센서 상태를 고객 화면에 전달
        photo = status_data.get('photo', False)
        self.customer_screen.update_photo_sensor(photo)
        
        # 관리자 화면 업데이트
        state = status_data.get('state', 'UNKNOWN')
        adv = self.advanced_screen
        
        adv.state_label.setText(state)
        
        state_colors = {
            'READY': 'green',
            'RUNNING': 'blue',
            'HOMING': 'orange',
            'CLEANING_DESCEND': 'cyan',
            'CLEANING_PROCESS': 'cyan',
            'CLEANING_RETURN_HOME': 'cyan',
            'WAITING_FOR_REMOVAL': 'purple',
            'ERROR_STATE': 'red'
        }
        color = state_colors.get(state, 'black')
        adv.state_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 14px;")
        
        adv.step_label.setText(str(status_data.get('step', '-')))
        
        # 센서 상태
        sensor_map = {
            'photo': status_data.get('photo', False),
            'dc_top': status_data.get('dc_top', False),
            'dc_bottom': status_data.get('dc_bottom', False),
            'stp_top': status_data.get('stp_top', False),
            'stp_bottom': status_data.get('stp_bottom', False)
        }
        
        for key, value in sensor_map.items():
            if key in adv.sensor_labels:
                label = adv.sensor_labels[key]
                if value:
                    label.setText("● ON")
                    label.setStyleSheet("color: green; font-weight: bold;")
                else:
                    label.setText("○ OFF")
                    label.setStyleSheet("color: gray;")
        
        self.statusBar.showMessage(f"상태: {state} | 단계: {status_data.get('step', 0)}")
    
    def add_log(self, message):
        """로그 추가"""
        # 관리자 화면의 로그에만 추가
        if hasattr(self.advanced_screen, 'log_text'):
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.advanced_screen.log_text.append(f"[{timestamp}] {message}")
            
            scrollbar = self.advanced_screen.log_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
    
    def closeEvent(self, event):
        """프로그램 종료"""
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    font = QFont("맑은 고딕", 10)
    app.setFont(font)
    
    gui = ArduinoControllerGUI()
    gui.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()