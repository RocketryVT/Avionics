# Ground-Station

## Dependencies

- [Qt 6](https://www.qt.io/download-qt-installer-oss) - version 6.9.2
  - Make sure to select "customize"
  - You do not need to install iOS or Android as we are not using Qt for mobile development.
  - Extensions: Qt WebEngine
  - Qt 6.9.2
    - Desktop
    - Additional Libraries:
      - Qt Charts
      - Qt Connectivity
      - Qt Data Visualization
      - Qt Graphs
      - Qt HTTP Server
      - Qt Image Formats
      - Qt Multimedia
      - Qt Network Authorization
      - Qt Positioning
      - Qt Protobuf
      - Qt Remote Objects
      - Qt Sensors
      - Qt Serial Bus
      - Qt Serial Port
      - Qt WebChannel
      - Qt WebSockets
      - Qt WebView
  - Build Tools:
    - CMake (If you are not on linux or macOS or already have it installed)
    - Ninja (If you are not on linux or macOS or already have it installed)
  - Make sure Qt Creator 17.0.0 is installed with the Qt 6.9.2 kit.
- [ArcGIS Runtime SDK for Qt](https://developers.arcgis.com/qt/downloads/#arcgis-maps-sdk-for-qt) - version 200.8.0
  - Place the `qt200.8.0` folder in the `include/linux`, `include/macos`, and `include/windows` directories for your platform.
  - The SDK is very large (around 1.5 GB), so ensure you have enough disk space. (This is why we don't include it in the repository.)

## Building

  [MacOS]
    - Add `-DCMAKE_MAKE_PROGRAM:FILEPATH=/opt/homebrew/bin/ninja`
