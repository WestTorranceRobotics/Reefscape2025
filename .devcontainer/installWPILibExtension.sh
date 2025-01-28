#!/usr/bin/env bash

# Use gradle dependency insight to get information on a wpilib package (contains version number). 
WPILIB_VERSION=$(./gradlew dependencyInsight --dependency edu.wpi.first | grep -iEo "[0-9]{4}\.[0-9]\.[0-9]" | head -1)

cd /opt 

wget -O vscode-wpilib.vsix "https://github.com/wpilibsuite/vscode-wpilib/releases/download/v${WPILIB_VERSION}/vscode-wpilib-${WPILIB_VERSION}.vsix"

code --install-extension /opt/vscode-wpilib.vsix