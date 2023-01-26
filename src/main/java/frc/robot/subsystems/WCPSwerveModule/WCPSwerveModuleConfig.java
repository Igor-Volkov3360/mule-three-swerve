// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WCPSwerveModule;

import com.ctre.phoenix.motorcontrol.InvertType;

/** Add your docs here. */
public class WCPSwerveModuleConfig {

  public final int m_turnMotorId;

  public final int m_driveMotorId;

  public final int m_analogZero;

  public final InvertType m_driveMotorInversion;

  public final boolean m_driveSensorInvertPhase;

  public WCPSwerveModuleConfig(
      int turnMotorId,
      int driveMotorId,
      int analogZero,
      InvertType driveMotorInversion,
      boolean driveSensorInvertPhase) {
    m_turnMotorId = turnMotorId;
    m_driveMotorId = driveMotorId;
    m_analogZero = analogZero;
    m_driveMotorInversion = driveMotorInversion;
    m_driveSensorInvertPhase = driveSensorInvertPhase;
  }
}
