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

public class Intake extends SubsystemBase {

  // Subsystem parameters
  private static final int kPivotLeadId = 0;
  private static final int kPivotFollowId = 0;
  private static final int kRollerId = 0;

  private static final double kNativeToRad = 1.0;
  private static final double kNominalVolt = 10.0;

  private static final double kNeutralRad = Math.toRadians(110.0);
  private static final double kHorizontalPercent = 0.0;

  private static final double kTargetTolRad = Math.toRadians(5.0);
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double kAngVelRad = Math.toRadians(90.0);
  private static final double kAngAccRed = Math.toRadians(90.0);

  private static final double kRetractRad = Math.toRadians(110.0);
  private static final double kExtendRad = Math.toRadians(45.0);
  private static final double kRollerPercent = 0.5;

  // Member objects
  private final CANSparkMax m_pivotLead = new CANSparkMax(kPivotLeadId, MotorType.kBrushless);
  private final CANSparkMax m_pivotFollow = new CANSparkMax(kPivotFollowId, MotorType.kBrushless);
  private final CANSparkMax m_roller = new CANSparkMax(kRollerId, MotorType.kBrushless);

  private final SparkMaxPIDController m_pid = m_pivotLead.getPIDController();
  private final RelativeEncoder m_encoder = m_pivotLead.getEncoder();

  // Process variables
  private double m_targetRad = kNeutralRad;

  /** Creates a new Intake. */
  public Intake() {

    m_roller.restoreFactoryDefaults();
    m_roller.burnFlash();

    m_pivotLead.restoreFactoryDefaults();
    m_pivotLead.enableVoltageCompensation(kNominalVolt);

    m_encoder.setPositionConversionFactor(kNativeToRad);
    m_encoder.setPosition(kNeutralRad);

    m_pid.setP(kP);
    m_pid.setD(kD);
    m_pid.setI(kI);

    m_pid.setSmartMotionMaxVelocity(kAngVelRad, 0);
    m_pid.setSmartMotionMaxAccel(kAngAccRed, 0);

    m_pivotLead.burnFlash();

    m_pivotFollow.restoreFactoryDefaults();
    m_pivotFollow.enableVoltageCompensation(kNominalVolt);
    m_pivotFollow.follow(m_pivotLead);
    m_pivotFollow.burnFlash();

    // Stop intake by default
    // this.setDefaultCommand(this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Set target to current when robot is disabled to prevent sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetRad = m_encoder.getPosition();
    }

    // Set reference in periodic to allow for arbitrary PID computation
    m_pid.setReference(m_targetRad, ControlType.kSmartMotion, 0, this.computeFeedForward());
  }

  /**
   * Compute arbitrary feed-forward for current intake position
   *
   * @return feed-forward (percent)
   */
  private double computeFeedForward() {
    return kHorizontalPercent * Math.cos(m_encoder.getPosition());
  }

  /**
   * Check if the intake has reached its target angle
   *
   * @return intake is on target
   */
  private boolean onTarget() {
    return Math.abs(m_encoder.getPosition() - m_targetRad) < kTargetTolRad;
  }

  /**
   * Set the roller speed and pivot to a given angle
   *
   * @param rollerPercent roller speed (percent)
   * @param pivotRad angle above horizontal (rad)
   * @return blocking command
   */
  private Command operate(double rollerPercent, double pivotRad) {
    return this.runOnce(() -> m_roller.set(rollerPercent))
        .andThen(this.run(() -> m_targetRad = pivotRad).until(this::onTarget));
  }

  /**
   * Start the roller and extend the intake
   *
   * @return blocking command
   */
  public Command extend() {
    return this.operate(kRollerPercent, kExtendRad);
  }

  /**
   * Stop the roller and retract the intake
   *
   * @return blocking command
   */
  public Command retract() {
    return this.operate(0.0, kRetractRad);
  }

  /**
   * Stop the roller
   *
   * @return instant command
   */
  public Command stop() {
    return this.runOnce(() -> m_roller.set(0.0));
  }
}
