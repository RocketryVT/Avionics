#!/bin/bash

# COPYRIGHT 2024 ESRI
#
# All rights reserved under the copyright laws of the United States
# and applicable international laws, treaties, and conventions.
#
# This material is licensed for use under the Esri Master License
# Agreement (MLA), and is bound by the terms of that agreement.
# You may redistribute and use this code without modification,
# provided you adhere to the terms of the MLA and include this
# copyright notice.
#
# See use restrictions at http://www.esri.com/legal/pdfs/mla_e204_e300/english
#
# For additional information, contact:
# Environmental Systems Research Institute, Inc.
# Attn: Contracts and Legal Services Department
# 380 New York Street
# Redlands, California, 92373
# USA

# configure.sh
# This script is used to configure the Linux installer for the ArcGIS Maps SDK for Qt.

function _display_help() {
  echo "================================================================================"
  echo "Usage: configure.sh [OPTION]..."
  echo
  echo "Description: configure.sh script is used to configure the ArcGIS Maps SDK for Qt and integrate documentation and templates into Qt Creator"
  echo
  echo " -np If specified the postinstall.sh script will not run. Default is to run the script."
  echo "          Optional"
  echo
  echo " -h Displays this help dialog."
  echo "          Optional"
  echo
  echo " -u Uninstall ArcGIS Maps SDK for Qt Linux."
  echo "          Optional"
  echo
  echo " -a, --accept-eula Accept the EULA for headless installs."
  echo "          Optional"
  echo
  echo "Example: To configure the ArcGIS Maps SDK for Qt for Linux"
  echo
  echo " ./configure.sh"
  echo
  echo "================================================================================"
  exit 0
}

UNINSTALL_SDK=false
RUN_POST_INSTALL=true
ACCEPT_AGREEMENT=false

# parse options
while [[ $# -gt 0 ]]; do
  case "$1" in
    -np)
      RUN_POST_INSTALL=false
      shift
      ;;
    -h)
      _display_help
      exit 0
      ;;
    -u)
      UNINSTALL_SDK=true
      shift
      ;;
    -a|--accept-eula)
      ACCEPT_AGREEMENT=true
      shift
      ;;
    *)
      echo "Invalid option: $1"
      _display_help
      exit 1
      ;;
  esac
done

LOG_FILE="/tmp/configure.log"

VERSION="200.8.0"

# Get the directory where the configure.sh script is located.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/qt$VERSION"

# Define the configuration file path.
CONFIG_FILE="$HOME/.config/EsriRuntimeQt/ArcGIS Runtime SDK for Qt $VERSION.ini"

function log_() {
  echo "$(date) - $1" >> "${LOG_FILE}"
}

# Function to replace the line that contains SDK_INSTALL_DIR = with the actual install directory in a given file.
function replace_install_dir() {
  local file=$1
  if [ -f "$file" ]; then
    if ! sed -i "s|^SDK_INSTALL_DIR = \".*\"|SDK_INSTALL_DIR = \"$SCRIPT_DIR\"|g" "$file"; then
    log_ "Error: Failed to replace SDK_INSTALL_DIR in $file."
    exit 1
    fi
  else
    log_ "Warning: $file does not exist."
  fi
}

function install_() {
  # Prompt user to accept the Master License Agreement before proceeding.
  if [ "$ACCEPT_AGREEMENT" = false ]; then
    echo "================================================================================"
    echo "IMPORTANT: You must accept the Esri Master License Agreement before proceeding."
    echo "The agreement can be found in the 'legal' folder as 'EULA.pdf'."
    echo "Do you accept the terms of the Master License Agreement? (yes/no)"
    read -r AGREEMENT_RESPONSE
    AGREEMENT_RESPONSE_LC=$(echo "$AGREEMENT_RESPONSE" | tr '[:upper:]' '[:lower:]')
    case "$AGREEMENT_RESPONSE_LC" in
      yes|y)
        # continue
        ;;
      no|n)
        echo "You must accept the Master License Agreement to proceed. Exiting."
        exit 1
        ;;
      *)
        echo "Invalid response. Please answer 'yes' or 'no'. Exiting."
        exit 1
        ;;
    esac
  fi

  # Create the directory if it does not exist.
  mkdir -p "$(dirname "$CONFIG_FILE")"

  # Create the file and update InstallDir.
  if ! echo "[Settings]" > "$CONFIG_FILE" || ! echo "InstallDir=\"$SCRIPT_DIR\"" >> "$CONFIG_FILE"; then
    log_ "Error: Failed to create or update the configuration file."
    exit 1
  fi

  # Replace <INSTALLDIR> in the specified files.
  replace_install_dir "$SCRIPT_DIR/sdk/ideintegration/esri_runtime_qt.pri"
  replace_install_dir "$SCRIPT_DIR/sdk/ideintegration/arcgis_runtime_qml_cpp.pri"
  log_ "Replaced the place holder <INSTALLDIR> in the specified files."

  # Run post install script.
  if [[ $RUN_POST_INSTALL == true ]]; then
    log_ "Running post install script"
    if ! bash "$SCRIPT_DIR"/tools/postInstall.sh; then
      log_ "Error: Failed to run post install script."
      exit 1
    fi
    log_ "Post install is successfull."
  fi

    # Print a message indicating configure script has completed.
  echo "Configuration is complete. Please refer to the log file for more details: $LOG_FILE"
}

function uninstall_() {
  # Uninstall ArcGIS Maps SDK for Qt
  log_ "Undoing post install script"
  if ! bash "$SCRIPT_DIR"/tools/postInstall.sh -uninstall; then
    log_ "Error: Failed to Undo post install script."
    exit 1
  fi
  log_ "Post install undo is successfull."

  log_ "Removing hidden ini file"
  if [ -f "$CONFIG_FILE" ]; then
    rm "$CONFIG_FILE"
  else
    echo "File does not exist, nothing to remove."
  fi

  # Print a message indicating configure script has completed.
  echo "Uninstall configuration is complete. Please refer to the log file for more details: $LOG_FILE"
}

if $UNINSTALL_SDK; then
  uninstall_
else
  install_
fi
