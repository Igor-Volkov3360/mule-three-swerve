// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public enum Position {
    Stored,
    Retracted,
    Pickup,
    Launch
  };

  // Subsystem parameters
  private static final int kPivot = 12;

  private static final int kEncoder = 7;
  private static final double kTurnPerRotation = 0.25;
  private static final double kOffset = 0.28;
  private static final double kP = 3.0;

  private static final double kStoredRad = 0.45;
  public final double kInsideRad = 0.30;
  private static final double kOutsideRad = 0.01;
  private static final double kLaunchRad = 0.2;
  private static double kDeadzoneRad = 0.05;

  // Member objects
  private final CANSparkMax m_pivot = new CANSparkMax(kPivot, MotorType.kBrushless);
  private final DutyCycleEncoder m_dutyEncoder = new DutyCycleEncoder(kEncoder);
  private final DigitalInput m_limitSwitch = new DigitalInput(8);

  // Process variables
  private double m_targetRad = kOutsideRad;
  private boolean isStopped = false;

  /** Creates a new Intake. */
  public Intake() {

    m_pivot.restoreFactoryDefaults();
    m_pivot.setIdleMode(IdleMode.kBrake);
    m_pivot.setInverted(true);

    m_dutyEncoder.setDistancePerRotation(kTurnPerRotation);
    m_dutyEncoder.reset();
    m_pivot.burnFlash();
  }

  @Override
  public void periodic() {
    // Set target to current when robot is disabled to prevent sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetRad = (getAngleRad());
    }

    // for pivot operation
    if (!isStopped) m_pivot.set(computePivotPercent());
    SmartDashboard.putBoolean("HAS CUBE :", hasCube());
  }

  /**
   * Gets the value of the left encoder between 0 and 0.45
   *
   * @return A value of 0 when the intake is down
   */
  private double getAngleRad() {
    return 1.0 - m_dutyEncoder.getAbsolutePosition() - kOffset;
  }

  /**
   * Compute pivot motor output to reach the target angle
   *
   * @return pivot motor percent
   */
  private double computePivotPercent() {
    final double errRad = (m_targetRad - this.getAngleRad()) * 10;
    double correctedSpeed;
    if (errRad > 0) correctedSpeed = 3 * kP * (errRad * errRad) / 10;
    else correctedSpeed = -(kP * (errRad * errRad) / 10);
    if (correctedSpeed > 0.5) correctedSpeed = 0.5;
    else if (correctedSpeed < -0.5) correctedSpeed = -0.5;
    return correctedSpeed;
  }

  /**
   * Set the target angle in rad corresponding to a position
   *
   * @param position target position
   */
  private void setAngleFor(Position position) {
    System.out.println("setting intake angle");
    switch (position) {
      case Stored:
        m_targetRad = kStoredRad;
        break;
      case Retracted:
        m_targetRad = kInsideRad;
        break;
      case Pickup:
        m_targetRad = kOutsideRad;
        break;
      case Launch:
        m_targetRad = kLaunchRad;
        break;
    }
  }

  /**
   * Sets the position of the intake
   *
   * @param position The position to shoot at
   * @return blocking command
   */
  public Command setAngle(Position position) {
    return this.run(() -> this.setAngleFor(position)).until(this::onTarget);
  }

  /**
   * Check if the intake is at the target angle
   *
   * @return intake is at the target angle
   */
  public boolean onTarget() {

    return Math.abs(m_targetRad - this.getAngleRad()) < kDeadzoneRad;
  }

  /**
   * Check if the intake holds cube
   *
   * @return cube is in intake
   */
  public boolean hasCube() {
    return !m_limitSwitch.get();
  }

  public double getPosition() {
    return m_targetRad;
  }

  public Command stop() {
    return this.runOnce(() -> isStopped = true);
  }
}
