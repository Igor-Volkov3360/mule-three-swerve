// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem to control the wrist holding the gripper */
public class Wrist extends SubsystemBase {

  // Subsystem parameters
  private static final int kId = 16;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double kVelDeg = 90.0;
  private static final double kAccDeg = 45.0;
  private static final double kTolDeg = 5.0;

  private static final double kHorizontalVolts = 0.0;
  private static final double kNominalVolt = 10.0;

  private static final double kNeutralDeg = -90.0;
  private static final double kNativeToDeg = 1.0;

  // Member objects
  private final CANSparkMax m_drive = new CANSparkMax(kId, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_drive.getEncoder();
  private final SparkMaxPIDController m_pid = m_drive.getPIDController();

  // Process variables
  private double m_targetDeg = kNeutralDeg;

  /** Creates a new Wrist. */
  public Wrist() {

    m_drive.restoreFactoryDefaults();
    m_drive.enableVoltageCompensation(kNominalVolt);

    m_encoder.setPositionConversionFactor(kNativeToDeg);
    m_encoder.setVelocityConversionFactor(kNativeToDeg);
    m_encoder.setPosition(kNeutralDeg);

    m_pid.setP(kP);
    m_pid.setI(kI);
    m_pid.setD(kD);
    m_pid.setSmartMotionMaxVelocity(kVelDeg, 0);
    m_pid.setSmartMotionMaxAccel(kAccDeg, 0);

    m_drive.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      // Set target to current when disabled
      m_targetDeg = m_encoder.getPosition();
    }

    final var FFVolt = kHorizontalVolts * Math.cos(Math.toRadians(m_encoder.getPosition()));
    m_pid.setReference(m_targetDeg, ControlType.kSmartMotion, 0, FFVolt, ArbFFUnits.kVoltage);
  }

  /**
   * Check if the wrist has reached its target
   *
   * @return current is within tolerance of target
   */
  private boolean onTarget() {
    return Math.abs(m_targetDeg - m_encoder.getPosition()) < kTolDeg;
  }

  /**
   * Rotate to a given angle
   *
   * @param angleDeg angle relative to horizontal (deg)
   * @return blocking command
   */
  public Command rotateTo(double angleDeg) {
    return this.run(() -> m_targetDeg = angleDeg).until(this::onTarget);
  }
}
