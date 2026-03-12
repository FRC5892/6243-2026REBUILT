#!/bin/bash
# Compile project
./gradlew build

# Run the simulation using the jar produced by the build (includes all WPILib deps)
java -cp build/libs/6243-2026REBUILT.jar frc.robot.simulations.shooter.ShotCalcSim