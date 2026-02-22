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

// Qt headers
#include <QApplication>
#include <QMessageBox>
#include <QIcon>

#include <QSurfaceFormat>

#include "ArcGISRuntimeEnvironment.h"

#include "ArcTest.h"

using namespace Esri::ArcGISRuntime;

int main(int argc, char *argv[])
{
#if defined(Q_OS_LINUX) && !defined(Q_OS_ANDROID)
    // Linux requires 3.2 OpenGL Context
    // in order to instance 3D symbols
    QSurfaceFormat fmt = QSurfaceFormat::defaultFormat();
    fmt.setVersion(3, 2);
    QSurfaceFormat::setDefaultFormat(fmt);
#endif

    QApplication application(argc, argv);
    
    // Set application properties
    application.setApplicationName("ArcGIS Ground Station");
    application.setApplicationDisplayName("ArcGIS Ground Station");
    application.setApplicationVersion("1.0.0");
    application.setOrganizationName("Your Company");
    application.setOrganizationDomain("yourcompany.com");
    
    // Set application icon (this will be used for taskbar, alt-tab, etc.)
    application.setWindowIcon(QIcon(":/resources/icons/app_icon.png"));

    // Use of ArcGIS location services, such as basemap styles, geocoding, and routing services,
    // requires an access token. For more information see
    // https://links.esri.com/arcgis-runtime-security-auth.

    // The following methods grant an access token:

    // 1. User authentication: Grants a temporary access token associated with a user's ArcGIS account.
    // To generate a token, a user logs in to the app with an ArcGIS account that is part of an
    // organization in ArcGIS Online or ArcGIS Enterprise.

    // 2. API key authentication: Get a long-lived access token that gives your application access to
    // ArcGIS location services. Go to the tutorial at https://links.esri.com/create-an-api-key.
    // Copy the API Key access token.

    const QString accessToken = QString(
        "AAPTxy8BH1VEsoebNVZXo8HurD4wz9CIN0zEJxlpF9FmAeSK9rrPg3G78mA2f4L-Ffun5ttmSjth0ED-"
        "ITeFI2QLCbBcUnA0RqNzWwEhxLLQFJkbWwTHLzLn4vmr5rramb3mwUmGPsudspOUvWZ3kIt3C4cqeI5hkX-"
        "X9zxCbF1LKGxGkilhe8rr_SizjFf8q_ZHFdm7oEiXts2lXBBPccy6Ktli8BKmwhmBGrx7w-Hwmj8.AT1_"
        "lCKWrTRc");

    if (accessToken.isEmpty()) {
        qWarning()
            << "Use of ArcGIS location services, such as the basemap styles service, requires"
            << "you to authenticate with an ArcGIS account or set the API Key property.";
    } else {
        ArcGISRuntimeEnvironment::setApiKey(accessToken);
    }

    // Production deployment of applications built with ArcGIS Maps SDK requires you to
    // license ArcGIS Maps SDK functionality. For more information see
    // https://links.esri.com/arcgis-runtime-license-and-deploy.

    // ArcGISRuntimeEnvironment::setLicense("runtimelite,1000,rud4397239387,none,5H80TK8ELBCSF5KHT234");

    //  use this code to check for initialization errors
    //  QObject::connect(ArcGISRuntimeEnvironment::instance(), &ArcGISRuntimeEnvironment::errorOccurred, [](const Error& error){
    //    QMessageBox msgBox;
    //    msgBox.setText(error.message);
    //    msgBox.exec();
    //  });

    //  if (ArcGISRuntimeEnvironment::initialize() == false)
    //  {
    //    application.quit();
    //    return 1;
    //  }

    // Set up ArcGIS Runtime environment
    // ArcGISRuntimeEnvironment::setInstallDirectory("C:/Program Files/ArcGIS SDKs/Qt200.8.0");

    ArcTest applicationWindow;
    applicationWindow.setMinimumWidth(800);
    applicationWindow.setMinimumHeight(600);
    applicationWindow.setWindowTitle("ArcGIS Ground Station - Rocket Telemetry");
    applicationWindow.setWindowIcon(QIcon(":/resources/icons/app_icon.png"));
    applicationWindow.show();

    return application.exec();
}
