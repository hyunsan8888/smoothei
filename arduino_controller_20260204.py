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
    QStatusBar, QMessageBox, QComboBox, QStackedWidget, QDialog,
    QDialogButtonBox
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


class CleaningConfirmDialog(QDialog):
    """세척 시작 확인 다이얼로그"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("세척 시작 확인")
        self.setFixedSize(420, 260)
        self.setModal(True)
        self.initUI()
    
    def initUI(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        layout.setContentsMargins(25, 25, 25, 20)

        # 경고 아이콘 + 제목
        title_layout = QHBoxLayout()
        icon_label = QLabel("🧼")
        icon_label.setStyleSheet("font-size: 36px;")
        title_label = QLabel("세척 작업을 시작합니다")
        title_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: #00838F;"
        )
        title_layout.addWidget(icon_label)
        title_layout.addWidget(title_label)
        title_layout.addStretch()
        layout.addLayout(title_layout)

        # 구분선
        line = QLabel()
        line.setFixedHeight(1)
        line.setStyleSheet("background-color: #e0e0e0;")
        layout.addWidget(line)

        # 체크리스트
        checklist = [
            "✔  컵이 시스템에서 완전히 제거되었습니까?",
            "✔  제조 작업이 완전히 종료되었습니까?",
            "✔  세척수 공급이 준비되었습니까?",
        ]
        for text in checklist:
            lbl = QLabel(text)
            lbl.setStyleSheet("font-size: 13px; color: #444; padding: 2px 0;")
            layout.addWidget(lbl)

        layout.addStretch()

        # 버튼
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(12)

        cancel_btn = QPushButton("취소")
        cancel_btn.setFixedHeight(42)
        cancel_btn.setStyleSheet("""
            QPushButton {
                font-size: 14px;
                background-color: #9E9E9E;
                color: white;
                border-radius: 8px;
                padding: 0 20px;
            }
            QPushButton:hover { background-color: #757575; }
        """)
        cancel_btn.clicked.connect(self.reject)

        confirm_btn = QPushButton("세척 시작")
        confirm_btn.setFixedHeight(42)
        confirm_btn.setStyleSheet("""
            QPushButton {
                font-size: 14px;
                font-weight: bold;
                background-color: #00BCD4;
                color: white;
                border-radius: 8px;
                padding: 0 20px;
            }
            QPushButton:hover { background-color: #0097A7; }
        """)
        confirm_btn.clicked.connect(self.accept)

        btn_layout.addStretch()
        btn_layout.addWidget(cancel_btn)
        btn_layout.addWidget(confirm_btn)
        layout.addLayout(btn_layout)


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
            self.show_buttons()
        elif not detected and self.photo_detected:
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
        
        # ── 세척 제어 (독립 실행 지원) ──
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
        """
        세척 제어 그룹 - 독립 실행
        Arduino 로 'CLEAN' 명령을 직접 전송하므로
        현재 시스템 상태(state)와 무관하게 언제든 실행 가능합니다.
        """
        group = QGroupBox("세척 제어")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #00BCD4;
                border-radius: 6px;
                margin-top: 6px;
                padding-top: 4px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                color: #00838F;
            }
        """)
        layout = QVBoxLayout()
        layout.setSpacing(8)

        # 안내 문구
        info_label = QLabel(
            "컵을 제거한 뒤 세척을 시작하세요.\n"
            "시스템 상태와 무관하게 단독 실행됩니다."
        )
        info_label.setStyleSheet("color: #555; font-size: 11px; font-weight: normal;")
        info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(info_label)

        # 세척 상태 표시 라벨
        self.cleaning_status_label = QLabel("대기 중")
        self.cleaning_status_label.setAlignment(Qt.AlignCenter)
        self.cleaning_status_label.setStyleSheet("""
            QLabel {
                font-size: 13px;
                font-weight: bold;
                color: #00838F;
                background-color: #E0F7FA;
                border-radius: 4px;
                padding: 4px;
            }
        """)
        layout.addWidget(self.cleaning_status_label)

        # 세척 시작 버튼
        self.cleaning_start_btn = QPushButton("🧼  세척 시작 (CLEAN)")
        self.cleaning_start_btn.setStyleSheet("""
            QPushButton {
                background-color: #00BCD4;
                color: white;
                font-weight: bold;
                padding: 15px;
                font-size: 14px;
                border-radius: 6px;
            }
            QPushButton:hover  { background-color: #0097A7; }
            QPushButton:pressed { background-color: #00838F; }
            QPushButton:disabled {
                background-color: #B2EBF2;
                color: #80DEEA;
            }
        """)
        self.cleaning_start_btn.setMinimumHeight(60)
        layout.addWidget(self.cleaning_start_btn)

        # 세척 중지 버튼 (STOP 재사용)
        self.cleaning_stop_btn = QPushButton("⏹  세척 중지 (STOP)")
        self.cleaning_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF7043;
                color: white;
                font-weight: bold;
                padding: 10px;
                font-size: 13px;
                border-radius: 6px;
            }
            QPushButton:hover  { background-color: #E64A19; }
            QPushButton:pressed { background-color: #BF360C; }
        """)
        self.cleaning_stop_btn.setMinimumHeight(44)
        layout.addWidget(self.cleaning_stop_btn)

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

    def set_cleaning_status(self, text, color="#00838F", bg="#E0F7FA"):
        """세척 상태 라벨 업데이트"""
        self.cleaning_status_label.setText(text)
        self.cleaning_status_label.setStyleSheet(f"""
            QLabel {{
                font-size: 13px;
                font-weight: bold;
                color: {color};
                background-color: {bg};
                border-radius: 4px;
                padding: 4px;
            }}
        """)


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
        
        # ── 세척 제어 (독립 실행) ──
        adv.cleaning_start_btn.clicked.connect(self.on_cleaning_start)
        adv.cleaning_stop_btn.clicked.connect(self.on_cleaning_stop)
        
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
        self.customer_screen.show_waiting_message()
    
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

    # ──────────────────────────────────────────────────────────────
    #  세척 제어 - 독립 실행
    #  시스템 상태(READY / RUNNING / WAITING …)와 무관하게
    #  관리자가 언제든 'CLEAN' 명령을 직접 전송할 수 있습니다.
    #
    #  세척 시퀀스 (타이머 기반 UI 표시):
    #    ① CLEAN 명령 전송
    #    ② DC모터 + 스테퍼 동시 하강 (descend_ms 동안)
    #    ③ 하강 완료 → 세척 동작 진행 (clean_ms 동안)
    #    ④ 홈 복귀 (return_ms 동안)
    #    ⑤ 세척 완료
    #
    #  ※ 실제 타이밍은 Arduino 펌웨어의 동작 시간에 맞춰
    #     DESCEND_MS / CLEAN_MS / RETURN_MS 값을 조정하세요.
    # ──────────────────────────────────────────────────────────────

    # 세척 단계별 소요 시간 (ms) - Arduino 펌웨어 타이밍에 맞게 조정
    DESCEND_MS = 3000   # DC모터 + 스테퍼 동시 하강 시간
    CLEAN_MS   = 8000   # 세척 동작 시간 (밸브/펌프/블렌더 등)
    RETURN_MS  = 3000   # 홈 복귀 시간

    def on_cleaning_start(self):
        """
        세척 시작 - 상태 조건 없이 단독 실행
        ① 확인 다이얼로그
        ② CLEAN 명령 전송
        ③ 타이머로 단계별 UI 상태 표시
           (하강 중 → 세척 중 → 복귀 중 → 완료)
        """
        dialog = CleaningConfirmDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            return

        adv = self.advanced_screen

        # 버튼 비활성화 (중복 실행 방지)
        adv.cleaning_start_btn.setEnabled(False)

        # ── 단계 1: DC모터 + 스테퍼 동시 하강 ──────────────────
        adv.set_cleaning_status(
            "① DC모터 + 스테퍼 하강 중 ↓",
            color="#1565C0", bg="#E3F2FD"
        )
        self.add_log("▶ 세척 시작 — DC모터 + 스테퍼 동시 하강")

        # Arduino 에 CLEAN 명령 전송 (하강 + 세척 전체 시퀀스 처리)
        self.send_command("CLEAN")

        # ── 단계 2: 하강 완료 후 세척 중 표시 (타이머) ──────────
        QTimer.singleShot(self.DESCEND_MS, self._cleaning_step_descend_done)

    def _cleaning_step_descend_done(self):
        """하강 완료 → 세척 동작 중 UI 표시"""
        adv = self.advanced_screen
        # 세척이 이미 중단된 경우 무시
        if adv.cleaning_start_btn.isEnabled():
            return
        adv.set_cleaning_status(
            "② 세척 동작 진행 중 🔄",
            color="#E65100", bg="#FFF3E0"
        )
        self.add_log("  ↳ 하강 완료 — 세척 동작 진행 중")

        # ── 단계 3: 세척 완료 후 홈 복귀 표시 (타이머) ──────────
        QTimer.singleShot(self.CLEAN_MS, self._cleaning_step_clean_done)

    def _cleaning_step_clean_done(self):
        """세척 완료 → 홈 복귀 중 UI 표시"""
        adv = self.advanced_screen
        if adv.cleaning_start_btn.isEnabled():
            return
        adv.set_cleaning_status(
            "③ 홈 복귀 중 ↑",
            color="#6A1B9A", bg="#F3E5F5"
        )
        self.add_log("  ↳ 세척 완료 — 홈 복귀 중")

        # ── 단계 4: 전체 완료 (타이머) ───────────────────────────
        QTimer.singleShot(self.RETURN_MS, self._cleaning_step_finished)

    def _cleaning_step_finished(self):
        """세척 전체 완료 — 버튼 복원 및 상태 초기화"""
        adv = self.advanced_screen
        if adv.cleaning_start_btn.isEnabled():
            return   # 이미 STOP으로 중단된 경우
        adv.set_cleaning_status(
            "✅ 세척 완료",
            color="#2E7D32", bg="#E8F5E9"
        )
        adv.cleaning_start_btn.setEnabled(True)
        self.add_log("■ 세척 프로세스 완료")

    def on_cleaning_stop(self):
        """세척 중지 - STOP 명령 전송"""
        reply = QMessageBox.question(
            self, '세척 중지',
            '세척을 중지하고 홈 위치로 복귀합니까?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.send_command("STOP")
            self.advanced_screen.cleaning_start_btn.setEnabled(True)
            self.advanced_screen.set_cleaning_status("대기 중")
            self.add_log("■ 세척 중지 (STOP 명령 전송)")

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

        # ── 세척 상태 자동 반영 (Arduino 상태값 우선) ───────────
        # Arduino 가 보내는 state 값으로 UI 를 실시간 동기화한다.
        # 타이머 기반 표시와 충돌하지 않도록 Arduino 값이 있으면
        # 타이머 표시를 덮어쓴다 (실제 하드웨어 상태가 우선).
        cleaning_step_map = {
            'CLEANING_DESCEND':     ("① DC모터 + 스테퍼 하강 중 ↓", "#1565C0", "#E3F2FD"),
            'CLEANING_PROCESS':     ("② 세척 동작 진행 중 🔄",      "#E65100", "#FFF3E0"),
            'CLEANING_RETURN_HOME': ("③ 홈 복귀 중 ↑",             "#6A1B9A", "#F3E5F5"),
        }
        is_cleaning = state.startswith('CLEANING')
        if is_cleaning:
            adv.cleaning_start_btn.setEnabled(False)
            step_text, step_color, step_bg = cleaning_step_map.get(
                state, (f"세척 중 ({state})", "#E65100", "#FFF3E0")
            )
            adv.set_cleaning_status(step_text, color=step_color, bg=step_bg)
        else:
            # 세척 상태가 아닐 때만 버튼 활성화 & 라벨 복원
            # (타이머 진행 중인 경우는 cleaning_start_btn 이 비활성이므로 구분 가능)
            adv.cleaning_start_btn.setEnabled(True)
            current_label = adv.cleaning_status_label.text()
            non_idle_labels = {"대기 중", "✅ 세척 완료"}
            if current_label not in non_idle_labels and not is_cleaning:
                adv.set_cleaning_status("대기 중")

        # 센서 상태
        sensor_map = {
            'photo':      status_data.get('photo', False),
            'dc_top':     status_data.get('dc_top', False),
            'dc_bottom':  status_data.get('dc_bottom', False),
            'stp_top':    status_data.get('stp_top', False),
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
