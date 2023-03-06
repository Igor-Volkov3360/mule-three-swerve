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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Subsystem parameters
  private static final int kPivotLeft = 9;
  private static final int kWheelsLeft = 17;
  private static final int kWheelsRight = 18;

  private static final int kEncoder = 7;
  private static final double kTurnPerRotation = 0.25;
  private static final double kOffset = 0.28;
  /* private static final double kUpperSoftLimit = 0.15;
  private static final double kLowwerSoftLimit = 0.03;

   private static final double kQuickBoiRight = 2.5;
  private static final double kSlowBoiRight = 2;
  private static final double kQuickBoiLeft = 2.5;
  private static final double kSlowBoiLeft = 2; */

  private static final double kUP = 0.18;
  private static final double kDOWN = 0.01;
  private static final double kConeIntake = 0.06;
  private static final double kWheelSpeed2nd = 0.5;
  private static final double kWheelSpeed3rd = 1.0;
  private double m_target = kUP;
  private double m_wheelSpeed = 0;
  private static double deadzone = 0.01;
  private static double angle = 70;
  private static double kNativeToAngle = 90 / 2048;

  // Member objects
  private final CANSparkMax m_pivot = new CANSparkMax(kPivotLeft, MotorType.kBrushless);
  private final CANSparkMax m_wheelsLeft = new CANSparkMax(kWheelsLeft, MotorType.kBrushless);
  private final CANSparkMax m_wheelsRight = new CANSparkMax(kWheelsRight, MotorType.kBrushless);
  private final DutyCycleEncoder m_dutyEncoder = new DutyCycleEncoder(kEncoder);
  private final DigitalInput m_limitSwitch = new DigitalInput(8);

  // Subsystem parameters

  /** Creates a new Intake. */
  public Intake() {

    m_pivot.restoreFactoryDefaults();
    m_pivot.setIdleMode(IdleMode.kBrake);
    m_pivot.setInverted(true);

    // m_wheelsLeft.setIdleMode(IdleMode.kCoast);

    //  m_wheelsRight.setIdleMode(IdleMode.kCoast);
    //  m_wheelsRight.setInverted(true);

    m_dutyEncoder.setDistancePerRotation(kTurnPerRotation);
    m_dutyEncoder.reset();

    m_pivot.burnFlash();
    // m_wheelsLeft.burnFlash();
    // m_wheelsRight.burnFlash();
  }

  @Override
  public void periodic() {

    limitSwitch();

    // Set target to current when robot is disabled to prevent sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_target = (getLeftEncoder());
    }

    System.out.println(limitSwitch());
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

    m_wheelsLeft.set(m_wheelSpeed);
    m_wheelsRight.set(m_wheelSpeed);
  }
  /**
   * Gets the value of the left encoder between 0 and 0.18
   *
   * @return A value of 0 when the intake is down
   */
  private double getLeftEncoder() {
    return 1 - m_dutyEncoder.getAbsolutePosition() - kOffset;
  }

  /**
   * This function sets the position of the intake
   *
   * @param position The position to shoot at
   * @return The intake in the right position
   */
  public Command setTarget(String position) {
    return this.runOnce(
        () -> {
          if (position == "up") {
            m_target = kUP;
          } else if (position == "down") {
            m_target = kDOWN;
          } else if (position == "cone") {
            m_target = kConeIntake;
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
            m_target = kUP;
          } else if (position == "down") {
            m_target = kDOWN;
          } else if (position == "cone") {
            m_target = kConeIntake;
          }
        });
  }

  public double motorSpeedLeft() {
    if (m_target == kConeIntake && !isOnTargetLeft()) return 0.06;
    else return m_target - getLeftEncoder();
  }

  private boolean isOnTargetLeft() {

    return (getLeftEncoder() < deadzone + kConeIntake && getLeftEncoder() > kConeIntake - deadzone);
  }

  /**
   * This function yeets the cubes
   *
   * @param level the desirerd level
   * @return the yeeting of a cube (handles the angle)
   */
  public Command yeet(String level) {
    return this.run(
        () -> {
          if (level == "second") {
            m_wheelSpeed = kWheelSpeed2nd;
            setTarget("second");
          } else if (level == "third") {
            m_wheelSpeed = kWheelSpeed3rd;
            setTarget("third");
          } else if (level == "stop") {
            m_wheelSpeed = 0;
            // setTarget("whatever");
          }
        });
  }

  public boolean limitSwitch() {
    return !m_limitSwitch.get();
  }

  public Command spinnyBoi() {
    return this.run(() -> m_wheelSpeed = 1);
  }

  public Command stop() {
    return this.runOnce(() -> m_wheelSpeed = 0);
  }
}
