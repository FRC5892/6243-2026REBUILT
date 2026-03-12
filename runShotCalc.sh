#!/bin/bash
set -euo pipefail

# Compile project first so the latest ShotCalcSim is in the jar.
./gradlew build -Dorg.gradle.java.home="/usr/lib/jvm/msopenjdk-current"

# Discover WPILib native lib directories needed by HAL/NT JNI at runtime.
LIB_PATH=$(
	find "$HOME/.gradle/caches" -type f -path "*/transforms/*" -name "*.so" -printf '%h\n' \
		| sort -u \
		| tr '\n' ':' \
		| sed 's/:$//'
)

if [[ -z "$LIB_PATH" ]]; then
	echo "Could not find WPILib native libraries under ~/.gradle/caches."
	exit 1
fi

# Pass through optional distance args, e.g. ./runShotCalc.sh 2.35 4.10
LD_LIBRARY_PATH="$LIB_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
	java -Djava.library.path="$LIB_PATH" \
		-cp build/libs/6243-2026REBUILT.jar \
		frc.robot.simulations.shooter.ShotCalcSim "$@"