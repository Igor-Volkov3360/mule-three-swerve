// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.HyperionSwerveModule.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Implements a single Hyperion custom swerve module */
public class HyperionSwerveModule implements SwerveModule {

  private final WPI_TalonSRX m_driveMotor;
  private final WPI_TalonSRX m_turnMotor;
  private final int m_analogZero;

  /**
   * Creates a swerve module instance
   *
   * @param driveMotorId Drive motor CAN id
   * @param turnMotorId Turn motor CAN id
   */
  public HyperionSwerveModule(
      int driveMotorId,
      int turnMotorId,
      int analogZero,
      boolean sensorOutOfPhase,
      InvertType driveInvertType) {

    m_analogZero = analogZero;

    m_driveMotor = new WPI_TalonSRX(driveMotorId);
    m_driveMotor.configFactoryDefault();

    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_driveMotor.config_kP(0, kDriveKp);
    m_driveMotor.config_kI(0, kDriveKi);
    m_driveMotor.config_kD(0, kDriveKd);
    m_driveMotor.config_kF(0, kDriveKf);
    m_driveMotor.config_IntegralZone(0, kDriveIZone);
    m_driveMotor.setInverted(driveInvertType);
    m_driveMotor.setSensorPhase(sensorOutOfPhase);

    m_turnMotor = new WPI_TalonSRX(turnMotorId);
    m_turnMotor.configFactoryDefault();

    m_turnMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    m_turnMotor.config_kP(0, kTurnKp);
    m_turnMotor.config_kI(0, kTurnKi);
    m_turnMotor.config_kD(0, kTurnKd);
    m_turnMotor.config_IntegralZone(0, kTurnIZone);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getSelectedSensorVelocity() * kTickToMeterPerS, this.getRotation());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getSelectedSensorPosition() * kTickToMeter, this.getRotation());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    var state = SwerveModuleState.optimize(desiredState, this.getRotation());
    m_driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond * kMeterPerSToTick);
    m_turnMotor.set(
        ControlMode.Position, state.angle.unaryMinus().getDegrees() * kDegToAnalog + m_analogZero);
  }

  /**
   * Gets the current rotation of the module.
   *
   * @return Current module rotation.
   */
  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(
            (m_turnMotor.getSelectedSensorPosition() - m_analogZero) * kAnalogToDeg)
        .unaryMinus();
  }
}
