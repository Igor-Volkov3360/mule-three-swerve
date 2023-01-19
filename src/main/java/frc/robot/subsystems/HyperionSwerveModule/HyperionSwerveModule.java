// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HyperionSwerveModule;

import static frc.robot.Constants.HyperionSwerveModule.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.SwerveModule;

/** Implements a single Hyperion custom swerve module */
public class HyperionSwerveModule implements SwerveModule {

  /** Driving motor */
  private final WPI_TalonSRX m_driveMotor;

  /** Turning motor */
  private final WPI_TalonSRX m_turnMotor;

  private final AnalogEncoder m_turnEncoder;

  /** Turning PID */
  private final PIDController m_turnPid = new PIDController(kTurnKp, kTurnKi, kTurnKd);

  /** Analog encoder value when module is at angle 0 */
  private final int m_analogZero;

  private final GenericEntry m_targetEntry;

  /**
   * Creates a module instance
   *
   * @param config configuration of this module
   */
  public HyperionSwerveModule(HyperionSwerveModuleConfig config) {

    m_analogZero = config.m_analogZero;

    m_driveMotor = new WPI_TalonSRX(config.m_driveMotorId);
    m_driveMotor.configFactoryDefault();

    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_driveMotor.config_kP(0, kDriveKp);
    m_driveMotor.config_kI(0, kDriveKi);
    m_driveMotor.config_kD(0, kDriveKd);
    m_driveMotor.config_kF(0, kDriveKf);
    m_driveMotor.config_IntegralZone(0, kDriveIZone);
    m_driveMotor.setInverted(config.m_driveMotorInversion);
    m_driveMotor.setSensorPhase(config.m_driveSensorInvertPhase);

    m_turnMotor = new WPI_TalonSRX(config.m_turnMotorId);
    m_turnMotor.configFactoryDefault();

    m_turnMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    m_turnMotor.config_kP(0, kTurnKp);
    m_turnMotor.config_kI(0, kTurnKi);
    m_turnMotor.config_kD(0, kTurnKd);
    m_turnMotor.config_IntegralZone(0, kTurnIZone);

    m_turnEncoder =
        new AnalogEncoder(config.m_turnEncoderChannel, kAnalogWrapThreshold, config.m_name);

    m_targetEntry = Shuffleboard.getTab("DT").add(config.m_name, -1.0).getEntry();
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

    final double turnTarget = state.angle.unaryMinus().getDegrees() * kDegToAnalog + m_analogZero;
    final double turnCommand = m_turnPid.calculate(m_turnEncoder.getAngle(), turnTarget);
    m_turnMotor.set(turnCommand / 1023.0);

    m_targetEntry.setDouble(state.angle.unaryMinus().getDegrees());
  }

  /**
   * Gets the current rotation of the module.
   *
   * @return Current module rotation.
   */
  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees((m_turnEncoder.getAngle() - m_analogZero) * kAnalogToDeg)
        .unaryMinus();
  }
}
