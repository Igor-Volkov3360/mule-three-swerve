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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public enum Position {
    Retracted,
    Pickup,
    Launch
  };

  public enum Level {
    Pickup,
    Preload,
    Second,
    Third,
    Stop
  };

  // Subsystem parameters
  private static final int kPivot = 12;
  private static final int kWheelsLeft = 17;
  private static final int kWheelsRight = 18;

  private static final int kEncoder = 7;
  private static final double kTurnPerRotation = 0.25;
  private static final double kOffset = 0.28;
  private static final double kP = 1.0;

  private static final double kInsideRad = 0.45;
  private static final double kOutsideRad = 0.01;
  private static final double kLaunchRad = 0.06;

  private static final double kWheelSpeedPreload = -0.25;
  private static final double kWheelSpeed2nd = 0.5;
  private static final double kWheelSpeed3rd = 1.0;
  private static final double kWheelSpeedPickup = -1.0;
  private static final double kWheelPreloadSec = 0.5;
  private static final double kWheelLaunchSec = 2.0;

  private static double kDeadzoneRad = 0.01;

  // Member objects
  private final CANSparkMax m_pivot = new CANSparkMax(kPivot, MotorType.kBrushless);
  private final CANSparkMax m_wheelsLeft = new CANSparkMax(kWheelsLeft, MotorType.kBrushless);
  private final CANSparkMax m_wheelsRight = new CANSparkMax(kWheelsRight, MotorType.kBrushless);
  private final DutyCycleEncoder m_dutyEncoder = new DutyCycleEncoder(kEncoder);
  private final DigitalInput m_limitSwitch = new DigitalInput(8);

  // Process variables
  private double m_targetRad = kInsideRad;
  private double m_wheelSpeed = 0;

  /** Creates a new Intake. */
  public Intake() {

    m_pivot.restoreFactoryDefaults();
    m_pivot.setIdleMode(IdleMode.kBrake);
    m_pivot.setInverted(true);

    m_dutyEncoder.setDistancePerRotation(kTurnPerRotation);
    m_dutyEncoder.reset();

    m_wheelsLeft.restoreFactoryDefaults();
    m_wheelsLeft.setIdleMode(IdleMode.kCoast);
    m_wheelsLeft.setInverted(true);

    m_wheelsRight.restoreFactoryDefaults();
    m_wheelsRight.setIdleMode(IdleMode.kCoast);

    m_pivot.burnFlash();
    m_wheelsLeft.burnFlash();
    m_wheelsRight.burnFlash();
  }

  @Override
  public void periodic() {

    // Set target to current when robot is disabled to prevent sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetRad = (getAngleRad());
    }

    // uncomment for pivot operation
    // m_pivot.set(computePivotPercent());

    // m_wheelsLeft.set(m_wheelSpeed);
    // m_wheelsRight.set(m_wheelSpeed);

    // System.out.printf(
    //     "Intake: target = %4.2f\tcurrent = %4.2f\t cube = %s\n",
    //     m_targetRad, this.getAngleRad(), this.hasCube() ? "true" : "false");
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
    final double errRad = m_targetRad - this.getAngleRad();
    return m_targetRad == kLaunchRad && !this.onTarget() ? 0.06 * Math.signum(errRad) : kP * errRad;
  }

  /**
   * Set the target angle in rad corresponding to a position
   *
   * @param position target position
   */
  private void setAngleFor(Position position) {
    switch (position) {
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
   * Set the target wheel speed in percent corresponding to a level
   *
   * @param level target level
   */
  private void setSpeedFor(Level level) {
    switch (level) {
      case Pickup:
        m_wheelSpeed = kWheelSpeedPickup;
        break;
      case Preload:
        m_wheelSpeed = kWheelSpeedPreload;
        break;
      case Second:
        m_wheelSpeed = kWheelSpeed2nd;
        break;
      case Third:
        m_wheelSpeed = kWheelSpeed3rd;
        break;
      case Stop:
        m_wheelSpeed = 0.0;
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
   * Holds a target angle for a given position
   *
   * @param position target position
   * @return forever command
   */
  public Command holdTarget(Position position) {
    return this.run(() -> this.setAngleFor(position));
  }

  /**
   * Hold the speed for a throw level
   *
   * @param level throwing level
   * @return forever command
   */
  public Command holdSpeed(Level level) {
    return this.run(() -> this.setSpeedFor(level));
  }

  /**
   * Check if the intake is at the target angle
   *
   * @return intake is at the target angle
   */
  private boolean onTarget() {

    return Math.abs(m_targetRad - this.getAngleRad()) < kDeadzoneRad;
  }

  /**
   * Launch a cube to a grid level
   *
   * @param level the desirerd level
   * @return the yeeting of a cube (handles the angle)
   */
  public Command launchToLevel(Level level) {
    return new SequentialCommandGroup(
        this.setAngle(Position.Launch),
        this.holdSpeed(Level.Preload).withTimeout(kWheelPreloadSec),
        this.holdSpeed(level).withTimeout(kWheelLaunchSec));
  }

  /**
   * Check if the intake holds cube
   *
   * @return cube is in intake
   */
  public boolean hasCube() {
    return !m_limitSwitch.get();
  }

  /**
   * Stop the intake wheels
   *
   * @return instant command
   */
  public Command stop() {
    return this.runOnce(() -> m_wheelSpeed = 0.0);
  }

  /**
   * Pickup a cube by lowering the intake and spinning until a cube is detected
   *
   * @return blocking command
   */
  public Command pickup() {
    return new SequentialCommandGroup(
        this.setAngle(Position.Pickup),
        this.holdSpeed(Level.Pickup).until(this::hasCube),
        this.stop());
  }
}
