// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WCPSwerveModule;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class WCPSwerveModule implements SwerveModule {

  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;

  WCPSwerveModule(WCPSwerveModuleConfig config) {

    m_turnMotor = new TalonFX(config.m_turnMotorId);
    m_turnMotor.configFactoryDefault();

    m_driveMotor = new TalonFX(config.m_driveMotorId);
    m_driveMotor.configFactoryDefault();
  }

  @Override
  public SwerveModuleState getState() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public SwerveModulePosition getPosition() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // TODO Auto-generated method stub

  }
}
