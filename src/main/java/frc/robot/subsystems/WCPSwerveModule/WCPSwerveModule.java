// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WCPSwerveModule;

import static frc.robot.Constants.WCPSwerveModule.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class WCPSwerveModule implements SwerveModule {

  private final int m_encoderZero;

  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;

  WCPSwerveModule(WCPSwerveModuleConfig config) {

    m_encoderZero = config.m_analogZero;

    m_driveMotor = new TalonFX(config.m_driveMotorId);
    m_driveMotor.configFactoryDefault();

    // m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    m_driveMotor.config_kP(0, kDriveKp);
    m_driveMotor.config_kI(0, kDriveKi);
    m_driveMotor.config_kD(0, kDriveKd);
    m_driveMotor.config_kF(0, kDriveKf);
    m_driveMotor.config_IntegralZone(0, kDriveIZone);
    m_driveMotor.setInverted(config.m_driveMotorInversion);
    m_driveMotor.setSensorPhase(config.m_driveSensorInvertPhase);

    m_turnMotor = new TalonFX(config.m_turnMotorId);
    m_turnMotor.configFactoryDefault();
    m_turnMotor.setInverted(InvertType.InvertMotorOutput);
    m_turnMotor.configVoltageCompSaturation(10);
    m_turnMotor.enableVoltageCompensation(true);

    // m_turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_turnMotor.config_kP(0, kTurnKp);
    m_turnMotor.config_kI(0, kTurnKi);
    m_turnMotor.config_kD(0, kTurnKd);
    m_turnMotor.config_IntegralZone(0, kTurnIZone);
  }

  @Override
  public SwerveModuleState getState() {

    return new SwerveModuleState(
        m_driveMotor.getSelectedSensorVelocity() * kTickToMeterPerS, this.getRotation());
  }

  @Override
  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(
        m_driveMotor.getSelectedSensorPosition() * kTickToMeter, this.getRotation());
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    var state = SwerveModuleState.optimize(desiredState, this.getRotation());
    m_driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond * kMeterPerSToTick);
    m_turnMotor.set(ControlMode.Position, state.angle.getDegrees() * kDegToAnalog + m_encoderZero);
    var rotationDelta = state.angle.minus(this.getRotation());
    var setpointDegrees = getEncoderDegrees() + rotationDelta.getDegrees();
    m_turnMotor.set(ControlMode.Position, setpointDegrees * kDegToAnalog + m_encoderZero);
  }

  private Rotation2d getRotation() {

    double encoderDegrees = getEncoderDegrees();

    return Rotation2d.fromDegrees(encoderDegrees % 360);
  }

  private double getEncoderDegrees() {

    return (m_turnMotor.getSelectedSensorPosition() - m_encoderZero) * kAnalogToDeg;
  }
}
