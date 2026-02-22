
# PyQt6 port of ArcTest C++ class for GroundStation
# Note: ArcGIS 3D Scene is replaced with a placeholder, as ArcGIS Runtime SDK is not available for Python.
# Serial port logic and telemetry dashboard are implemented using PyQt6 and pyserial.


import sys
import math
import time
from PyQt6.QtWidgets import (
	QApplication, QMainWindow, QWidget, QLabel, QGridLayout, QVBoxLayout, QHBoxLayout, QGroupBox,
	QComboBox, QPushButton, QMenuBar, QToolBar, QStatusBar, QFileDialog, QMessageBox, QMenu, QWidgetAction
)
from PyQt6.QtCore import Qt, QTimer, QDateTime, QTime, QUrl
from PyQt6.QtGui import QColor, QFont

from PyQt6.QtWebEngineWidgets import QWebEngineView

# Protobuf integration
import sys as _sys
_sys.path.append('/Users/gregory/Downloads/pico-freertos/Libs/Protobufs')
import data_pb2

try:
	import serial
	import serial.tools.list_ports
except ImportError:
	serial = None

MAX_ALTITUDE = 3048.0  # 10,000 feet in meters
GROUND_LEVEL = 0.0

class ArcTest(QMainWindow):
	def __init__(self):
		super().__init__()
		self.setWindowTitle("GroundStation - PyQt6 Port")

		# Simulation parameters
		self.m_launchLat = 34.0522
		self.m_launchLon = -118.2437
		self.m_initialVelocityX = 0.0005  # deg/s (simulated)
		self.m_initialVelocityY = 0.0003
		self.m_initialVelocityZ = 200.0  # m/s
		self.m_gravity = -9.81
		self.m_timeStep = 0.1
		self.m_currentTime = 0.0
		self.m_maxAltitude = 0.0
		self.m_maxVelocity = 0.0
		self.m_flightStarted = False
		self.m_flightStartTime = QTime.currentTime()
		self.m_isRecording = False
		self.m_logFile = None
		self.m_logStream = None
		self.m_useRealTelemetry = False
		self.m_isConnected = False

		# UI setup
		self.setupMenuBar()
		self.setupConnectionBar()
		self.setupTelemetryDashboard()

		# Timer for simulation/animation
		self.m_animationTimer = QTimer(self)
		self.m_animationTimer.timeout.connect(self.updateRocketPosition)
		self.m_animationTimer.start(100)

		# Serial port
		self.serial_port = None

	def setupMenuBar(self):
		# Only add the menu bar for other menus if needed, but serial settings will be in-app now
		self.menuBar = QMenuBar(self)
		self.setMenuBar(self.menuBar)
		# (You can add other menus here if needed)


	# No longer needed, serial settings are now in-app
	pass

	def setupConnectionBar(self):
		self.connectionToolbar = QToolBar("Connection", self)
		self.addToolBar(self.connectionToolbar)

		self.connectionToolbar.addWidget(QLabel("Port:"))
		self.m_serialPortCombo = QComboBox(self)
		self.m_serialPortCombo.setMinimumWidth(150)
		self.connectionToolbar.addWidget(self.m_serialPortCombo)

		self.refreshButton = QPushButton("Refresh", self)
		self.refreshButton.clicked.connect(self.refreshSerialPorts)
		self.connectionToolbar.addWidget(self.refreshButton)

		self.connectionToolbar.addSeparator()

		# Single Connect/Disconnect button
		self.m_connectToggleButton = QPushButton("Connect", self)
		self.m_connectToggleButton.clicked.connect(self.toggleConnection)
		self.connectionToolbar.addWidget(self.m_connectToggleButton)

		self.connectionToolbar.addSeparator()

		self.m_connectionStatus = QLabel("Disconnected", self)
		self.m_connectionStatus.setStyleSheet("color: red; font-weight: bold;")
		self.connectionToolbar.addWidget(self.m_connectionStatus)

		self.refreshSerialPorts()

	def setupTelemetryDashboard(self):
		self.m_centralWidget = QWidget(self)
		self.m_mainLayout = QGridLayout(self.m_centralWidget)
		self.m_mainLayout.setSpacing(8)
		self.m_mainLayout.setContentsMargins(8,8,8,8)

		# Add Serial Settings Toolbar as a second row (below connection bar)
		self.serialSettingsToolbar = QToolBar("Serial Settings", self)
		self.addToolBarBreak()  # Forces the next toolbar to a new row
		self.addToolBar(Qt.ToolBarArea.TopToolBarArea, self.serialSettingsToolbar)
		# Baud rate
		self.serialSettingsToolbar.addWidget(QLabel("Baud:"))
		self.m_baudRateCombo = QComboBox(self)
		for br in [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]:
			self.m_baudRateCombo.addItem(str(br), br)
		self.m_baudRateCombo.setCurrentText("115200")
		self.serialSettingsToolbar.addWidget(self.m_baudRateCombo)
		# Data bits
		self.serialSettingsToolbar.addWidget(QLabel("Data bits:"))
		self.m_dataBitsCombo = QComboBox(self)
		for bits in [5, 6, 7, 8]:
			self.m_dataBitsCombo.addItem(str(bits), bits)
		self.m_dataBitsCombo.setCurrentText("8")
		self.serialSettingsToolbar.addWidget(self.m_dataBitsCombo)
		# Parity
		self.serialSettingsToolbar.addWidget(QLabel("Parity:"))
		self.m_parityCombo = QComboBox(self)
		for name, val in [("None", 0), ("Even", 2), ("Odd", 3), ("Mark", 4), ("Space", 5)]:
			self.m_parityCombo.addItem(name, val)
		self.m_parityCombo.setCurrentText("None")
		self.serialSettingsToolbar.addWidget(self.m_parityCombo)
		# Stop bits
		self.serialSettingsToolbar.addWidget(QLabel("Stop bits:"))
		self.m_stopBitsCombo = QComboBox(self)
		for name, val in [("1", 1), ("1.5", 3), ("2", 2)]:
			self.m_stopBitsCombo.addItem(name, val)
		self.m_stopBitsCombo.setCurrentText("1")
		self.serialSettingsToolbar.addWidget(self.m_stopBitsCombo)
		# Connect signals
		self.m_baudRateCombo.currentIndexChanged.connect(self.applySerialSettings)
		self.m_dataBitsCombo.currentIndexChanged.connect(self.applySerialSettings)
		self.m_parityCombo.currentIndexChanged.connect(self.applySerialSettings)
		self.m_stopBitsCombo.currentIndexChanged.connect(self.applySerialSettings)

		# Top left: CesiumJS 3D globe in QWebEngineView
		if QWebEngineView is not None:
			self.m_webView = QWebEngineView(self)
			import os
			html_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "cesium_rocket_demo.html"))
			# Enable local file and remote URL access and JavaScript
			from PyQt6.QtWebEngineCore import QWebEngineSettings
			settings = self.m_webView.settings()
			settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
			settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
			settings.setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
			self.m_webView.setUrl(QUrl.fromLocalFile(html_path))
			self.m_mainLayout.addWidget(self.m_webView, 0, 0)
		else:
			self.m_mapLabel = QLabel("[3D Map Placeholder: PyQt6-WebEngine not installed]", self)
			mapFont = QFont()
			mapFont.setPointSize(18)
			mapFont.setBold(True)
			self.m_mapLabel.setFont(mapFont)
			self.m_mapLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
			self.m_mainLayout.addWidget(self.m_mapLabel, 0, 0)

		# Top right: GPS label
		self.m_gpsLabel = QLabel("Lat: --\nLon: --\nAlt: --", self)
		gpsFont = QFont()
		gpsFont.setPointSize(14)
		self.m_gpsLabel.setFont(gpsFont)
		self.m_gpsLabel.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
		self.m_mainLayout.addWidget(self.m_gpsLabel, 0, 1)

		# Bottom left: Real-time sensor data
		sensorWidget = QWidget(self)
		sensorLayout = QGridLayout(sensorWidget)
		self.m_altitudeLabel = QLabel("Altitude: -- m", self)
		self.m_velocityLabel = QLabel("Velocity: -- m/s", self)
		self.m_accelerationLabel = QLabel("Acceleration: -- m/s²", self)
		self.m_temperatureLabel = QLabel("Temperature: -- °C", self)
		self.m_pressureLabel = QLabel("Pressure: -- hPa", self)
		self.m_batteryLabel = QLabel("Battery: -- V", self)
		sensorFont = QFont()
		sensorFont.setPointSize(12)
		sensorFont.setBold(True)
		for i, label in enumerate([
			self.m_altitudeLabel, self.m_velocityLabel, self.m_accelerationLabel,
			self.m_temperatureLabel, self.m_pressureLabel, self.m_batteryLabel
		]):
			label.setFont(sensorFont)
			sensorLayout.addWidget(label, i, 0)
		self.m_mainLayout.addWidget(sensorWidget, 1, 0)

		# Bottom right: System status and controls
		controlWidget = QWidget(self)
		controlLayout = QVBoxLayout(controlWidget)

		# Data logging controls
		loggingGroup = QGroupBox("Data Logging", self)
		loggingLayout = QHBoxLayout(loggingGroup)
		self.m_recordButton = QPushButton("Start Recording", self)
		self.m_recordButton.setStyleSheet("QPushButton { background-color: #ff4444; color: white; font-weight: bold; }")
		self.m_recordButton.clicked.connect(self.toggleRecording)
		self.m_recordingStatus = QLabel("Not Recording", self)
		self.m_recordingStatus.setStyleSheet("color: red; font-weight: bold;")
		loggingLayout.addWidget(self.m_recordButton)
		loggingLayout.addWidget(self.m_recordingStatus)
		controlLayout.addWidget(loggingGroup)

		# Mission status
		missionGroup = QGroupBox("Mission Status", self)
		missionLayout = QVBoxLayout(missionGroup)
		self.m_flightTimeLabel = QLabel("Flight Time: 00:00", self)
		self.m_apogeeLabel = QLabel("Max Altitude: -- m", self)
		self.m_maxVelocityLabel = QLabel("Max Velocity: -- m/s", self)
		missionLayout.addWidget(self.m_flightTimeLabel)
		missionLayout.addWidget(self.m_apogeeLabel)
		missionLayout.addWidget(self.m_maxVelocityLabel)
		controlLayout.addWidget(missionGroup)

		self.m_mainLayout.addWidget(controlWidget, 1, 1)

		self.m_mainLayout.setRowStretch(0, 2)
		self.m_mainLayout.setRowStretch(1, 1)
		self.m_mainLayout.setColumnStretch(0, 2)
		self.m_mainLayout.setColumnStretch(1, 1)

		self.setCentralWidget(self.m_centralWidget)

	def refreshSerialPorts(self):
		self.m_serialPortCombo.clear()
		if serial is None:
			self.m_serialPortCombo.addItem("pyserial not installed", "")
			return
		ports = list(serial.tools.list_ports.comports())
		for port in ports:
			self.m_serialPortCombo.addItem(f"{port.device} ({port.description})", port.device)

	def applySerialSettings(self):
		# Only applies if serial port is open
		if self.serial_port is None or not self.serial_port.is_open:
			return
		try:
			baud = int(self.m_baudRateCombo.currentText())
			self.serial_port.baudrate = baud
			self.serial_port.bytesize = int(self.m_dataBitsCombo.currentText())
			parity_map = {0: 'N', 2: 'E', 3: 'O', 4: 'M', 5: 'S'}
			self.serial_port.parity = parity_map.get(self.m_parityCombo.currentData(), 'N')
			stop_map = {1: 1, 2: 2, 3: 1.5}
			self.serial_port.stopbits = stop_map.get(self.m_stopBitsCombo.currentData(), 1)
		except Exception as e:
			print(f"Serial settings error: {e}")

	def toggleConnection(self):
		if not self.m_isConnected:
			# Connect
			if serial is None:
				QMessageBox.critical(self, "Error", "pyserial is not installed.")
				return
			port = self.m_serialPortCombo.currentData()
			if not port:
				QMessageBox.warning(self, "No Port", "Please select a serial port.")
				return
			try:
				self.serial_port = serial.Serial(port, timeout=0.1)
				self.applySerialSettings()
				self.m_isConnected = True
				self.m_useRealTelemetry = True
				self.m_connectToggleButton.setText("Disconnect")
				self.m_connectionStatus.setText("Connected")
				self.m_connectionStatus.setStyleSheet("color: green; font-weight: bold;")
			except Exception as e:
				QMessageBox.critical(self, "Connection Error", str(e))
		else:
			# Disconnect
			if self.serial_port:
				self.serial_port.close()
			self.m_isConnected = False
			self.m_useRealTelemetry = False
			self.m_connectToggleButton.setText("Connect")
			self.m_connectionStatus.setText("Disconnected")
			self.m_connectionStatus.setStyleSheet("color: red; font-weight: bold;")

	def toggleRecording(self):
		if not self.m_isRecording:
			fname, _ = QFileDialog.getSaveFileName(self, "Save Log File", "", "CSV Files (*.csv)")
			if not fname:
				return
			try:
				self.m_logFile = open(fname, 'w')
				self.m_logStream = self.m_logFile
				self.m_isRecording = True
				self.m_recordButton.setText("Stop Recording")
				self.m_recordingStatus.setText("Recording")
				self.m_recordingStatus.setStyleSheet("color: green; font-weight: bold;")
			except Exception as e:
				QMessageBox.critical(self, "File Error", str(e))
		else:
			if self.m_logFile:
				self.m_logFile.close()
			self.m_isRecording = False
			self.m_recordButton.setText("Start Recording")
			self.m_recordingStatus.setText("Not Recording")
			self.m_recordingStatus.setStyleSheet("color: red; font-weight: bold;")

	def updateGpsLabel(self, lat, lon, alt):
		self.m_gpsLabel.setText(f"Lat: {lat:.6f}\nLon: {lon:.6f}\nAlt: {alt:.1f} m")

	def updateRocketPosition(self):
		# If using real telemetry, try to read from serial
		if self.m_useRealTelemetry and self.serial_port and self.serial_port.is_open and data_pb2 is not None:
			try:
				import struct
				# Only read if at least 2 bytes are available for header
				if self.serial_port.in_waiting >= 2:
					header = self.serial_port.read(2)
					if len(header) == 2:
						msg_len = struct.unpack('<H', header)[0]
						# Only read if the full message is available
						if self.serial_port.in_waiting >= msg_len:
							msg_bytes = self.serial_port.read(msg_len)
							if len(msg_bytes) == msg_len:
								try:
									pkt = data_pb2.TrackerPacket()
									pkt.ParseFromString(msg_bytes)
									self.handleProtobufPacket(pkt)
								except Exception as e:
									print(f"Protobuf parse error: {e}")
				return
			except Exception as e:
				print(f"Serial read error: {e}")
				return
	def handleProtobufPacket(self, pkt):
		# Update dashboard from TrackerPacket
		# Only handle GPS packets for now
		if pkt.packet_type == data_pb2.TrackerPacket.GPS and pkt.HasField('gps'):
			gps = pkt.gps
			lat = gps.lat
			lon = gps.lon
			alt = gps.alt
			self.updateGpsLabel(lat, lon, alt)
			self.m_altitudeLabel.setText(f"Altitude: {alt:.1f} m")
			# You can add more fields as needed
		# TODO: handle other packet types (IMU, Baro, etc.)

		# Simulate projectile motion
		t = self.m_currentTime
		earthRadius = 6371000.0
		latChange = (self.m_initialVelocityY * t) / earthRadius * (180.0 / math.pi)
		lonChange = (self.m_initialVelocityX * t) / (earthRadius * math.cos(self.m_launchLat * math.pi / 180.0)) * (180.0 / math.pi)
		newLat = self.m_launchLat + latChange
		newLon = self.m_launchLon + lonChange
		newAlt = GROUND_LEVEL + (self.m_initialVelocityZ * t) + (0.5 * self.m_gravity * t * t)
		if newAlt > MAX_ALTITUDE:
			newAlt = MAX_ALTITUDE
		if newAlt < GROUND_LEVEL:
			newAlt = GROUND_LEVEL
			self.m_currentTime = 0
			self.m_maxAltitude = 0.0
			self.m_maxVelocity = 0.0
			self.m_flightStarted = False
			self.updateGpsLabel(self.m_launchLat, self.m_launchLon, GROUND_LEVEL)
			return
		self.updateGpsLabel(newLat, newLon, newAlt)
		currentVel = math.sqrt(self.m_initialVelocityX**2 + self.m_initialVelocityY**2 + (self.m_initialVelocityZ + self.m_gravity * t)**2)
		currentAcc = 9.81
		self.m_altitudeLabel.setText(f"Altitude: {newAlt:.1f} m")
		self.m_velocityLabel.setText(f"Velocity: {currentVel:.1f} m/s")
		self.m_accelerationLabel.setText(f"Acceleration: {currentAcc:.1f} m/s²")
		self.m_temperatureLabel.setText(f"Temperature: {25.0 - newAlt/1000.0 * 6.5:.1f} °C")
		self.m_pressureLabel.setText(f"Pressure: {1013.0 * pow(1 - 0.0065 * newAlt / 288.15, 5.255):.1f} hPa")
		self.m_batteryLabel.setText(f"Battery: {12.0 - t * 0.01:.2f} V")
		if newAlt > self.m_maxAltitude:
			self.m_maxAltitude = newAlt
			self.m_apogeeLabel.setText(f"Max Altitude: {self.m_maxAltitude:.1f} m")
		if currentVel > self.m_maxVelocity:
			self.m_maxVelocity = currentVel
			self.m_maxVelocityLabel.setText(f"Max Velocity: {self.m_maxVelocity:.1f} m/s")
		if not self.m_flightStarted and newAlt > 10:
			self.m_flightStarted = True
			self.m_flightStartTime = QTime.currentTime()
		if self.m_flightStarted:
			elapsed = self.m_flightStartTime.secsTo(QTime.currentTime())
			self.m_flightTimeLabel.setText(f"Flight Time: {elapsed//60:02d}:{elapsed%60:02d}")
		if self.m_isRecording and self.m_logStream:
			self.m_logStream.write(f"{QDateTime.currentDateTime().toString('yyyy-MM-dd hh:mm:ss.zzz')},{newLat},{newLon},{newAlt},{currentVel},{currentAcc},{25.0 - newAlt/1000.0 * 6.5},{1013.0 * pow(1 - 0.0065 * newAlt / 288.15, 5.255)},{12.0 - t * 0.01}\n")
			self.m_logStream.flush()
		self.m_currentTime += self.m_timeStep
		if self.m_currentTime > 60.0:
			self.m_currentTime = 0
			self.m_maxAltitude = 0.0
			self.m_maxVelocity = 0.0
			self.m_flightStarted = False
			self.updateGpsLabel(self.m_launchLat, self.m_launchLon, GROUND_LEVEL)

	def parseTelemetryData(self, dataStr):
		# Example: "LAT:34.0522,LON:-118.2437,ALT:1500,VEL:250,ACC:9.8,TEMP:25,PRES:1013,BAT:12.5"
		try:
			parts = dataStr.strip().split(',')
			vals = {k: 0.0 for k in ['LAT','LON','ALT','VEL','ACC','TEMP','PRES','BAT']}
			for part in parts:
				if ':' in part:
					k, v = part.split(':', 1)
					k = k.strip().upper()
					try:
						vals[k] = float(v)
					except ValueError:
						pass
			lat, lon, alt, vel, acc, temp, pres, bat = (
				vals['LAT'], vals['LON'], vals['ALT'], vals['VEL'], vals['ACC'], vals['TEMP'], vals['PRES'], vals['BAT']
			)
			self.m_altitudeLabel.setText(f"Altitude: {alt:.1f} m")
			self.m_velocityLabel.setText(f"Velocity: {vel:.1f} m/s")
			self.m_accelerationLabel.setText(f"Acceleration: {acc:.1f} m/s²")
			self.m_temperatureLabel.setText(f"Temperature: {temp:.1f} °C")
			self.m_pressureLabel.setText(f"Pressure: {pres:.1f} hPa")
			self.m_batteryLabel.setText(f"Battery: {bat:.2f} V")
			if alt > self.m_maxAltitude:
				self.m_maxAltitude = alt
				self.m_apogeeLabel.setText(f"Max Altitude: {self.m_maxAltitude:.1f} m")
			if vel > self.m_maxVelocity:
				self.m_maxVelocity = vel
				self.m_maxVelocityLabel.setText(f"Max Velocity: {self.m_maxVelocity:.1f} m/s")
			if not self.m_flightStarted and alt > 10:
				self.m_flightStarted = True
				self.m_flightStartTime = QTime.currentTime()
			if self.m_flightStarted:
				elapsed = self.m_flightStartTime.secsTo(QTime.currentTime())
				self.m_flightTimeLabel.setText(f"Flight Time: {elapsed//60:02d}:{elapsed%60:02d}")
			if self.m_isRecording and self.m_logStream:
				self.m_logStream.write(f"{QDateTime.currentDateTime().toString('yyyy-MM-dd hh:mm:ss.zzz')},{lat},{lon},{alt},{vel},{acc},{temp},{pres},{bat}\n")
				self.m_logStream.flush()
			self.updateGpsLabel(lat, lon, alt)
		except Exception as e:
			print(f"Telemetry parse error: {e}")

def main():
	app = QApplication(sys.argv)
	window = ArcTest()
	window.resize(1000, 700)
	window.show()
	sys.exit(app.exec())

if __name__ == "__main__":
	main()
