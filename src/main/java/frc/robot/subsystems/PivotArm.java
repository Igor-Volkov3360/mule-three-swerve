// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotArm extends SubsystemBase {

  // Subsystem parameters
  private static final int kLeadId = 16;
  private static final double kNativeToRad = 1.0;
  private static final double kNominalVolt = 10.0;

  private static final double kNeutralRad = Math.toRadians(-90.0);
  private static final double kHorizontalPercent = 0.0;

  private static final double kTargetTolRad = Math.toRadians(5.0);
  private static final double kP = 0.01;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double kAngVelRad = Math.toRadians(45.0);
  private static final double kAngAccRed = Math.toRadians(45.0);

  // Member objects
  private final CANSparkMax m_pivot = new CANSparkMax(kLeadId, MotorType.kBrushless);
  private final SparkMaxPIDController m_pid = m_pivot.getPIDController();
  private final RelativeEncoder m_encoder = m_pivot.getEncoder();

  // Process variables
  private double m_targetRad = kNeutralRad;

  /** Creates a new PivotArm. */
  public PivotArm() {

    m_pivot.restoreFactoryDefaults();

    m_pivot.enableVoltageCompensation(kNominalVolt);

    m_encoder.setPositionConversionFactor(kNativeToRad);
    m_encoder.setPosition(kNeutralRad);

    m_pid.setP(kP);
    m_pid.setD(kD);
    m_pid.setI(kI);

    m_pid.setSmartMotionMaxVelocity(kAngVelRad, 0);
    m_pid.setSmartMotionMaxAccel(kAngAccRed, 0);

    m_pivot.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Set target to current when robot is disabled to prevent sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetRad = m_encoder.getPosition();
    }

    // Set reference in periodic to allow for arbitrary feed-forward computation
    m_pid.setReference(m_targetRad, ControlType.kSmartMotion, 0, this.computeFeedForward());

    System.out.println(m_pivot.getAppliedOutput());
  }

  /**
   * Compute arbitrary feed-forward for current arm position
   *
   * @return feed-forward (percent)
   */
  private double computeFeedForward() {
    return kHorizontalPercent * Math.cos(m_encoder.getPosition());
  }

  /**
   * Check if the arm has reached its target angle
   *
   * @return arm is on target
   */
  private boolean onTarget() {
    return Math.abs(m_encoder.getPosition() - m_targetRad) < kTargetTolRad;
  }

  /**
   * Pivot to a given angle above the horizontal
   *
   * @param degrees target angle (degrees)
   * @return blocking command
   */
  public Command pivotTo(double degrees) {
    return this.run(() -> m_targetRad = Math.toRadians(degrees)).until(this::onTarget);
  }
}
