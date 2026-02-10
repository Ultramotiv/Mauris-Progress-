#!/usr/bin/env python3
"""
MAURIS - Modular Adaptive Universal Robotic Intelligent System
Therapy Control Interface for Ubuntu 22.04
"""

import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QLabel, QPushButton, QLineEdit, QComboBox, 
    QFrame, QMessageBox, QSpacerItem, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, Property
from PySide6.QtGui import QPixmap, QPalette, QColor, QLinearGradient, QPainter, QBrush


class PulsingDot(QFrame):
    """Animated pulsing status indicator"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(12, 12)
        self._opacity = 1.0
        
        # Setup animation
        self.animation = QPropertyAnimation(self, b"opacity")
        self.animation.setDuration(2000)
        self.animation.setStartValue(1.0)
        self.animation.setEndValue(0.5)
        self.animation.setEasingCurve(QEasingCurve.InOutQuad)
        self.animation.setLoopCount(-1)  # Infinite loop
        self.animation.start()
    
    def get_opacity(self):
        return self._opacity
    
    def set_opacity(self, value):
        self._opacity = value
        self.update()
    
    opacity = Property(float, get_opacity, set_opacity)
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw pulsing circle
        color = QColor(255, 152, 0)
        color.setAlphaF(self._opacity)
        painter.setBrush(QBrush(color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(0, 0, 12, 12)


class StyledButton(QPushButton):
    """Custom styled button with hover effects"""
    def __init__(self, text, primary=True, parent=None):
        super().__init__(text, parent)
        self.primary = primary
        self.update_style()
        self.setCursor(Qt.PointingHandCursor)
    
    def update_style(self):
        if self.primary:
            self.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #44b8a6, stop:1 #2d9d8b);
                    color: white;
                    border: none;
                    padding: 14px 24px;
                    font-size: 16px;
                    font-weight: 600;
                    border-radius: 8px;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #55c9b7, stop:1 #3eae9c);
                }
                QPushButton:pressed {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #33a795, stop:1 #268c7a);
                }
            """)
        else:
            self.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #2d2d2d, stop:1 #1a1a1a);
                    color: white;
                    border: 1px solid rgba(68, 184, 166, 0.3);
                    padding: 16px;
                    font-size: 16px;
                    font-weight: 600;
                    border-radius: 8px;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #44b8a6, stop:1 #2d9d8b);
                    border: 1px solid #44b8a6;
                }
                QPushButton:pressed {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #33a795, stop:1 #268c7a);
                }
            """)


class SectionFrame(QFrame):
    """Styled section container"""
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            SectionFrame {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(26, 58, 52, 230), stop:1 rgba(45, 90, 79, 230));
                border: 1px solid rgba(68, 184, 166, 0.3);
                border-radius: 16px;
                padding: 20px;
            }
        """)
        
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(20, 20, 20, 20)
        self.layout.setSpacing(15)
        
        # Section title
        title_layout = QHBoxLayout()
        title_dot = QFrame()
        title_dot.setFixedSize(12, 12)
        title_dot.setStyleSheet("""
            background-color: white;
            border-radius: 6px;
        """)
        
        title_label = QLabel(title)
        title_label.setStyleSheet("""
            color: white;
            font-size: 20px;
            font-weight: 600;
        """)
        
        title_layout.addWidget(title_dot)
        title_layout.addWidget(title_label)
        title_layout.addStretch()
        
        self.layout.addLayout(title_layout)


class MAURISMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MAURIS - Therapy Control Interface")
        self.setMinimumSize(1000, 700)
        
        # Setup main widget with gradient background
        main_widget = QWidget()
        main_widget.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #000000, stop:0.5 #1a3a34, stop:0.75 #2d5a4f, stop:1 #44b8a6);
                color: white;
                font-family: 'Ubuntu', 'Segoe UI', sans-serif;
            }
            QLineEdit, QComboBox {
                background: rgba(0, 0, 0, 0.3);
                border: 1px solid rgba(68, 184, 166, 0.3);
                border-radius: 8px;
                padding: 12px;
                color: white;
                font-size: 15px;
            }
            QLineEdit:focus, QComboBox:focus {
                border: 1px solid #44b8a6;
            }
            QComboBox::drop-down {
                border: none;
                width: 30px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid white;
                margin-right: 10px;
            }
            QComboBox QAbstractItemView {
                background: #1a3a34;
                color: white;
                selection-background-color: #44b8a6;
                border: 1px solid #44b8a6;
            }
            QLabel {
                color: white;
            }
        """)
        
        self.setCentralWidget(main_widget)
        
        # Main layout
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)
        
        # Header
        self.create_header(main_layout)
        
        # Main content grid
        content_grid = QGridLayout()
        content_grid.setSpacing(20)
        
        # Robot Connection Section
        self.create_robot_section(content_grid)
        
        # Patient Information Section
        self.create_patient_section(content_grid)
        
        main_layout.addLayout(content_grid)
        
        # Therapy Mode Section
        self.create_therapy_section(main_layout)
        
        main_layout.addStretch()
    
    def create_header(self, layout):
        """Create header with logo and title"""
        header = QFrame()
        header.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(26, 58, 52, 204), stop:1 rgba(45, 90, 79, 204));
                border: 1px solid rgba(68, 184, 166, 0.3);
                border-radius: 16px;
                padding: 16px;
            }
        """)
        
        header_layout = QHBoxLayout(header)
        
        # Logo (placeholder)
        logo_label = QLabel("LOGO")
        logo_label.setFixedSize(120, 60)
        logo_label.setAlignment(Qt.AlignCenter)
        logo_label.setStyleSheet("""
            background: rgba(68, 184, 166, 0.2);
            border-radius: 8px;
            font-weight: bold;
        """)
        header_layout.addWidget(logo_label)
        
        # Title section
        title_widget = QWidget()
        title_layout = QVBoxLayout(title_widget)
        title_layout.setSpacing(3)
        title_layout.setContentsMargins(0, 0, 0, 0)
        
        title1 = QLabel("MAURIS")
        title1.setAlignment(Qt.AlignCenter)
        title1.setStyleSheet("font-size: 18px; font-weight: 700; letter-spacing: 1px;")
        
        title2 = QLabel("Modular Adaptive Universal Robotic Intelligent System")
        title2.setAlignment(Qt.AlignCenter)
        title2.setStyleSheet("font-size: 18px; font-weight: 700; letter-spacing: 1px;")
        
        subtitle = QLabel("Reversing Disabilities Completely")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet("font-size: 14px; color: #a0a0a0; font-weight: 500;")
        
        title_layout.addWidget(title1)
        title_layout.addWidget(title2)
        title_layout.addWidget(subtitle)
        
        header_layout.addWidget(title_widget, 1)
        
        # Spacer for balance
        spacer = QWidget()
        spacer.setFixedWidth(120)
        header_layout.addWidget(spacer)
        
        layout.addWidget(header)
    
    def create_robot_section(self, grid):
        """Create robot connection section"""
        section = SectionFrame("Robot Connection")
        
        # Status box
        status_box = QFrame()
        status_box.setStyleSheet("""
            QFrame {
                background: rgba(0, 0, 0, 0.3);
                border: 1px solid rgba(68, 184, 166, 0.2);
                border-radius: 12px;
                padding: 15px;
            }
        """)
        
        status_layout = QHBoxLayout(status_box)
        
        # Pulsing dot
        self.status_dot = PulsingDot()
        status_layout.addWidget(self.status_dot)
        
        # Status text
        status_text_widget = QWidget()
        status_text_layout = QVBoxLayout(status_text_widget)
        status_text_layout.setContentsMargins(0, 0, 0, 0)
        status_text_layout.setSpacing(3)
        
        self.status_title = QLabel("Status: Connecting...")
        self.status_title.setStyleSheet("font-size: 16px; font-weight: 600;")
        
        self.status_desc = QLabel("Attempting to connect to robot")
        self.status_desc.setStyleSheet("font-size: 13px; color: #a0a0a0;")
        
        status_text_layout.addWidget(self.status_title)
        status_text_layout.addWidget(self.status_desc)
        
        status_layout.addWidget(status_text_widget, 1)
        
        section.layout.addWidget(status_box)
        
        # Progress Report button
        report_btn = StyledButton("Progress Report")
        report_btn.clicked.connect(self.generate_report)
        section.layout.addWidget(report_btn)
        
        grid.addWidget(section, 0, 0)
    
    def create_patient_section(self, grid):
        """Create patient information section"""
        section = SectionFrame("Patient Information")
        
        # Name
        name_label = QLabel("Full Name:")
        name_label.setStyleSheet("font-weight: 500; font-size: 14px; margin-bottom: 5px;")
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Enter patient name")
        
        section.layout.addWidget(name_label)
        section.layout.addWidget(self.name_input)
        
        # Age
        age_label = QLabel("Age:")
        age_label.setStyleSheet("font-weight: 500; font-size: 14px; margin-bottom: 5px;")
        self.age_input = QLineEdit()
        self.age_input.setPlaceholderText("Enter age")
        
        section.layout.addWidget(age_label)
        section.layout.addWidget(self.age_input)
        
        # Gender
        gender_label = QLabel("Gender:")
        gender_label.setStyleSheet("font-weight: 500; font-size: 14px; margin-bottom: 5px;")
        self.gender_combo = QComboBox()
        self.gender_combo.addItems(["Select gender", "Male", "Female", "Other"])
        
        section.layout.addWidget(gender_label)
        section.layout.addWidget(self.gender_combo)
        
        # Diagnosis
        diagnosis_label = QLabel("Diagnosis:")
        diagnosis_label.setStyleSheet("font-weight: 500; font-size: 14px; margin-bottom: 5px;")
        self.diagnosis_input = QLineEdit()
        self.diagnosis_input.setPlaceholderText("Enter diagnosis")
        
        section.layout.addWidget(diagnosis_label)
        section.layout.addWidget(self.diagnosis_input)
        
        section.layout.addStretch()
        
        grid.addWidget(section, 0, 1)
    
    def create_therapy_section(self, layout):
        """Create therapy mode selection section"""
        section = SectionFrame("Therapy Mode Selection")
        
        # Mode buttons grid
        modes_grid = QGridLayout()
        modes_grid.setSpacing(15)
        
        modes = ["Active Mode", "Passive Mode", "Guided Mode", 
                 "Resistive Mode", "Game Mode"]
        
        for i, mode in enumerate(modes):
            btn = StyledButton(mode, primary=False)
            btn.clicked.connect(lambda checked, m=mode: self.select_mode(m))
            row = i // 2
            col = i % 2
            modes_grid.addWidget(btn, row, col)
        
        section.layout.addLayout(modes_grid)
        layout.addWidget(section)
    
    def generate_report(self):
        """Generate progress report"""
        name = self.name_input.text().strip()
        if not name:
            QMessageBox.warning(self, "Missing Information", 
                              "Please enter patient information first!")
            return
        
        QMessageBox.information(self, "Progress Report", 
                              f"Generating progress report for {name}...\n\n"
                              "Report will be available shortly.")
    
    def select_mode(self, mode):
        """Handle therapy mode selection"""
        reply = QMessageBox.question(self, "Mode Selected", 
                                    f"Exercise Selected: {mode}\n\n"
                                    "Preparing exercise session...\n\n"
                                    "Would you like to proceed to exercise selection?",
                                    QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # Here you would navigate to exercise selection window
            QMessageBox.information(self, "Next Step", 
                                  f"Opening exercise selection for {mode}...")


def main():
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle("Fusion")
    
    window = MAURISMainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()