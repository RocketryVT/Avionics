// Copyright 2025 ESRI
//
// All rights reserved under the copyright laws of the United States
// and applicable international laws, treaties, and conventions.
//
// You may freely redistribute and use this sample code, with or
// without modification, provided you include the original copyright
// notice and use restrictions.
//
// See the Sample code usage restrictions document for further information.
//

#ifndef ARCTEST_H
#define ARCTEST_H

#include <Point.h>
namespace Esri::ArcGISRuntime {
class SceneGraphicsView;
class GraphicsOverlay;
class Graphic;
class PolylineBuilder;
} // namespace Esri::ArcGISRuntime

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QGridLayout>
#include <QWidget>
#include <QMenuBar>
#include <QStatusBar>
#include <QComboBox>
#include <QPushButton>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QToolBar>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFile>
#include <QTextStream>
#include <QTime>
#include <QDateTime>

class ArcTest : public QMainWindow
{
    Q_OBJECT
public:
    explicit ArcTest(QWidget *parent = nullptr);
    ~ArcTest() override;

public slots:
    void updateRocketPosition();
    void updateGpsLabel(double lat, double lon, double alt);
    void refreshSerialPorts();
    void connectToDevice();
    void disconnectFromDevice();
    void readSerialData();
    void toggleRecording();
    void parseTelemtryData(const QByteArray& data);
    void updateRocketPositionFromTelemetry(double lat, double lon, double alt);

private:
    void setupConnectionBar();
    void setupTelemetryDashboard();
    void applySerialSettings();
    
    Esri::ArcGISRuntime::SceneGraphicsView *m_sceneView = nullptr;
    Esri::ArcGISRuntime::GraphicsOverlay *m_graphicsOverlay = nullptr;
    Esri::ArcGISRuntime::Graphic *m_rocketGraphic = nullptr;
    Esri::ArcGISRuntime::Graphic *m_trajectoryGraphic = nullptr;
    Esri::ArcGISRuntime::PolylineBuilder *m_trajectoryBuilder = nullptr;
    QTimer *m_animationTimer = nullptr;
    
    // Animation parameters
    double m_currentTime = 0.0;
    double m_timeStep = 0.1;  // seconds per update
    
    // Trajectory parameters
    double m_launchLat = 34.0522;   // Los Angeles latitude
    double m_launchLon = -118.2437; // Los Angeles longitude
    double m_initialVelocityX = 500.0; // m/s eastward
    double m_initialVelocityY = 300.0; // m/s northward  
    double m_initialVelocityZ = 800.0; // m/s upward
    double m_gravity = -9.81; // m/s²
    
    // 3D Trajectory tracking
    QList<Esri::ArcGISRuntime::Point> m_trajectoryPoints;

    // Camera orbit parameters
    double m_cameraOrbitAngle = 0.0; // radians
    double m_cameraOrbitRadius = 30000.0; // meters (30km from launch)
    double m_cameraOrbitSpeed = 0.01; // radians per update
    double m_cameraOrbitHeight = 20000.0; // meters above ground

    // UI elements
    QLabel* m_gpsLabel = nullptr;
    QWidget* m_bottomLeftWidget = nullptr;
    QWidget* m_bottomRightWidget = nullptr;
    QGridLayout* m_mainLayout = nullptr;
    QWidget* m_centralWidget = nullptr;
    
    // Connection UI
    QMenuBar* m_menuBar = nullptr;
    QComboBox* m_serialPortCombo = nullptr;
    QPushButton* m_connectButton = nullptr;
    QPushButton* m_disconnectButton = nullptr;
    QLabel* m_connectionStatus = nullptr;
    QComboBox* m_baudRateCombo = nullptr;
    QComboBox* m_dataBitsCombo = nullptr;
    QComboBox* m_parityCombo = nullptr;
    QComboBox* m_stopBitsCombo = nullptr;
    
    // Serial communication
    QSerialPort* m_serialPort = nullptr;
    bool m_isConnected = false;
    
    // Telemetry labels
    QLabel* m_altitudeLabel = nullptr;
    QLabel* m_velocityLabel = nullptr;
    QLabel* m_accelerationLabel = nullptr;
    QLabel* m_temperatureLabel = nullptr;
    QLabel* m_pressureLabel = nullptr;
    QLabel* m_batteryLabel = nullptr;
    
    // Mission tracking
    QLabel* m_flightTimeLabel = nullptr;
    QLabel* m_apogeeLabel = nullptr;
    QLabel* m_maxVelocityLabel = nullptr;
    
    // Data logging
    QPushButton* m_recordButton = nullptr;
    QLabel* m_recordingStatus = nullptr;
    QFile* m_logFile = nullptr;
    QTextStream* m_logStream = nullptr;
    bool m_isRecording = false;
    
    // Mission data
    double m_maxAltitude = 0.0;
    double m_maxVelocity = 0.0;
    QTime m_flightStartTime;
    bool m_flightStarted = false;
    bool m_useRealTelemetry = false;

};

#endif // ARCTEST_H
