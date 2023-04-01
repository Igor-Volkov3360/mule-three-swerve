// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WCPSwerveModule;

import static frc.robot.Constants.WCPSwerveModule.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class WCPSwerveModule implements SwerveModule {

  // Subsystem parameters
  public static final double kNominalVolt = 10.0;
  public static final double kTickPerRotation = 28000.0;

  public static final double kPwmPeriod = 1.0 / 244.0;
  public static final double kPwmDutyMin = 1e-6 / kPwmPeriod;
  public static final double kPwmDutyMax = 4096e-6 / kPwmPeriod;

  private double m_encoderZero = 0.0;
  private boolean m_homed = false;
  private final double m_configZero;

  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final DutyCycleEncoder m_magEncoder;

  private final GenericEntry m_absAngleEntry;
  private final GenericEntry m_encOkEntry;

  WCPSwerveModule(WCPSwerveModuleConfig config) {

    m_magEncoder = new DutyCycleEncoder(config.m_magEncoderChannel);
    m_magEncoder.setDistancePerRotation(kTickPerRotation);
    m_magEncoder.setDutyCycleRange(kPwmDutyMin, kPwmDutyMax);

    m_driveMotor = new TalonFX(config.m_driveMotorId);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.setNeutralMode(NeutralMode.Brake);

    m_driveMotor.config_kP(0, kDriveKp);
    m_driveMotor.config_kI(0, kDriveKi);
    m_driveMotor.config_kD(0, kDriveKd);
    m_driveMotor.config_kF(0, kDriveKf);
    m_driveMotor.config_IntegralZone(0, kDriveIZone);
    m_driveMotor.configVoltageCompSaturation(kNominalVolt);
    m_driveMotor.enableVoltageCompensation(true);

    m_turnMotor = new TalonFX(config.m_turnMotorId);
    m_turnMotor.configFactoryDefault();
    m_turnMotor.setInverted(InvertType.InvertMotorOutput);
    m_turnMotor.configVoltageCompSaturation(kNominalVolt);
    m_turnMotor.enableVoltageCompensation(true);
    m_turnMotor.setNeutralMode(NeutralMode.Brake);

    m_turnMotor.config_kP(0, kTurnKp);
    m_turnMotor.config_kI(0, kTurnKi);
    m_turnMotor.config_kD(0, kTurnKd);
    m_turnMotor.config_IntegralZone(0, kTurnIZone);

    m_configZero = config.m_analogZero;

    final var absAngleEntryName = String.format("Abs Ch %d", config.m_magEncoderChannel);
    m_absAngleEntry =
        Shuffleboard.getTab("WCP Swerve Module").add(absAngleEntryName, 0.0).getEntry();

    final var encOkEntryName = String.format("Enc %d OK", config.m_magEncoderChannel);
    m_encOkEntry = Shuffleboard.getTab("Vitals").add(encOkEntryName, false).getEntry();
  }

  @Override
  public void periodic() {
    m_absAngleEntry.setDouble(m_magEncoder.getDistance());
    m_encOkEntry.setBoolean(m_magEncoder.isConnected());

    if (!m_homed) {
      // Home on first periodic loop so sensors are fully initialized
      m_encoderZero =
          m_configZero + m_turnMotor.getSelectedSensorPosition() - m_magEncoder.getDistance();
      m_homed = true;
    }
  }

  private Rotation2d getRotation() {

    double encoderDegrees = getEncoderDegrees();
    return Rotation2d.fromDegrees(encoderDegrees % 360);
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

  private double getEncoderDegrees() {

    return (m_turnMotor.getSelectedSensorPosition() - m_encoderZero) * kAnalogToDeg;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    var state = SwerveModuleState.optimize(desiredState, this.getRotation());
    var rotationDelta = state.angle.minus(this.getRotation());
    var setpointDegrees = getEncoderDegrees() + rotationDelta.getDegrees();

    m_driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond * kMeterPerSToTick);
    m_turnMotor.set(ControlMode.Position, setpointDegrees * kDegToAnalog + m_encoderZero);
  }
}
