// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Subsystem parameters
  private static final int kPivotLeft = 9;
  private static final int kPivotRight = 10;
  private static final int kLeftEncoder = 7;
  private static final int KRightEncoder = 8;
  private static final double kTurnPerRotation = 0.25;
  private static final double kOffsetLeft = 0.28;
  private static final double kOffsetRight = 0.17;
  /* private static final double kUpperSoftLimit = 0.15;
  private static final double kLowwerSoftLimit = 0.03;

   private static final double kQuickBoiRight = 2.5;
  private static final double kSlowBoiRight = 2;
  private static final double kQuickBoiLeft = 2.5;
  private static final double kSlowBoiLeft = 2; */

  private static final double kUP = 0.18;
  private static final double kDOWN = 0.01;
  private static final double kConeIntake = 0.06;
  private double m_targetLeft = kUP;
  private double m_targetRight = kUP;
  private static double deadzone = 0.01;
  // Member objects
  private final CANSparkMax m_pivotLeft = new CANSparkMax(kPivotLeft, MotorType.kBrushless);
  private final CANSparkMax m_pivotRight = new CANSparkMax(kPivotRight, MotorType.kBrushless);
  private final DutyCycleEncoder m_dutyEncoderLeft = new DutyCycleEncoder(kLeftEncoder);
  private final DutyCycleEncoder m_dutyEncoderRight = new DutyCycleEncoder(KRightEncoder);
  /** Creates a new Intake. */
  public Intake() {

    m_pivotLeft.restoreFactoryDefaults();
    m_pivotLeft.setIdleMode(IdleMode.kBrake);
    m_pivotLeft.setInverted(true);

    m_pivotRight.restoreFactoryDefaults();
    m_pivotRight.setIdleMode(IdleMode.kBrake);

    m_dutyEncoderLeft.setDistancePerRotation(kTurnPerRotation);
    m_dutyEncoderRight.setDistancePerRotation(kTurnPerRotation);
    m_dutyEncoderLeft.reset();
    m_dutyEncoderRight.reset();

    m_pivotLeft.burnFlash();
    m_pivotRight.burnFlash();
  }

  @Override
  public void periodic() {

    // Set target to current when robot is disabled to prevent sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetLeft = (getLeftEncoder() + getRightEncoder()) / 2;
      m_targetRight = (getLeftEncoder() + getRightEncoder()) / 2;
    }
    /*
    if (isLeftInFastRange() && isRightInFastRange()) {
      m_pivotLeft.set(motorSpeedLeft() * kQuickBoiLeft);
      m_pivotRight.set(motorSpeedRight() * kQuickBoiRight);
    } else {
      m_pivotLeft.set(motorSpeedLeft() * kSlowBoiLeft);
      m_pivotRight.set(motorSpeedRight() * kSlowBoiRight);
    }
    */

    // System.out.print("target " + m_targetLeft);
  }
  /**
   * Gets the value of the left encoder between 0 and 0.18
   *
   * @return A value of 0 when the intake is down
   */
  private double getLeftEncoder() {
    return 1 - m_dutyEncoderLeft.getAbsolutePosition() - kOffsetLeft;
  }

  /**
   * Gets the value of the right encoder between 0 and 0.18
   *
   * @return A value of 0 when the intake is down
   */
  private double getRightEncoder() {
    return m_dutyEncoderRight.getAbsolutePosition() - kOffsetRight;
  }

  /* private boolean isLeftInFastRange() {
    return getLeftEncoder() > kLowwerSoftLimit && getLeftEncoder() < kUpperSoftLimit;
  }

  private boolean isRightInFastRange() {
    return getRightEncoder() > kLowwerSoftLimit && getRightEncoder() < kUpperSoftLimit;
  } */

  public Command setTarget(String position) {
    return this.runOnce(
        () -> {
          if (position == "up") {
            m_targetLeft = kUP;
            m_targetRight = kUP;
          } else if (position == "down") {
            m_targetLeft = kDOWN;
            m_targetRight = kDOWN;
          } else if (position == "cone") {
            m_targetLeft = kConeIntake;
            m_targetRight = kConeIntake;
          }
        });
  }

  /**
   * This function holds an intake position
   *
   * @param position either "up", "down", "cone"
   * @return blocking command that holds a position
   */
  public Command holdTarget(String position) {
    return this.run(
        () -> {
          if (position == "up") {
            m_targetLeft = kUP;
            m_targetRight = kUP;
          } else if (position == "down") {
            m_targetLeft = kDOWN;
            m_targetRight = kDOWN;
          } else if (position == "cone") {
            m_targetLeft = kConeIntake;
            m_targetRight = kConeIntake;
          }
        });
  }

  public double motorSpeedLeft() {
    if (m_targetLeft == kConeIntake && !isOnTargetLeft()) return 0.06;
    else return m_targetLeft - getLeftEncoder();
  }

  public double motorSpeedRight() {
    if (m_targetRight == kConeIntake && !isOnTargetRight()) return 0.06;
    else return m_targetRight - getRightEncoder();
  }

  private boolean isOnTargetLeft() {

    return (getLeftEncoder() < deadzone + kConeIntake && getLeftEncoder() > kConeIntake - deadzone);
  }

  private boolean isOnTargetRight() {

    return (getRightEncoder() < deadzone + kConeIntake
        && getRightEncoder() > kConeIntake - deadzone);
  }
}
