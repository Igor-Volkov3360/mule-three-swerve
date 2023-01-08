// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.HyperionSwerveModule.*;

import edu.wpi.first.math.geometry.Translation2d;

/** Implements the swerve factory for Hyperion custom module. */
public class HyperionSwerveModuleFactory implements SwerveModuleFactory {

  @Override
  public SwerveModule[] createModules() {
    var modules = new HyperionSwerveModule[kDriveMotorIds.length];
    for (int i = 0; i < modules.length; ++i) {
      modules[i] =
          new HyperionSwerveModule(
              kDriveMotorIds[i],
              kTurnMotorIds[i],
              kAnalogZero[i],
              kSensorPhases[i],
              kInvertType[i]);
    }

    return modules;
  }

  @Override
  public Translation2d[] getLocations() {
    return kLocations;
  }
}
