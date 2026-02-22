ArcGIS Maps SDK for Qt - Linux SDK Configuration
======================================================

To configure the ArcGIS Maps SDK for Qt for use, run:

  ./configure.sh

You will be prompted to accept the Esri Master License Agreement.

By default, this script will also execute postInstall.sh, which integrates documentation and templates into Qt Creator.

Optional: To skip running postInstall.sh, use the -np flag:

  ./configure.sh -np

To uninstall and remove all installed components, use the -u flag:

  ./configure.sh -u

This will ensure that all SDK files, documentation, and templates are removed from your system.

For detailed setup and installation instructions, visit:
https://developers.arcgis.com/qt/install-and-set-up/

Note: If you move or rename the extracted directory, please re-run ./configure.sh.