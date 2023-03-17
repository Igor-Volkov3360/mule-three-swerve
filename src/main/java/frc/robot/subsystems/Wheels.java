// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheels extends SubsystemBase {

  public enum WheelLevel {
    Pickup,
    Preload,
    First,
    Second,
    Third,
    Hold,
    Stop
  };

  private static final int kWheelsLeft = 17;
  private static final int kWheelsRight = 18;

  private static final double kWheelSpeedPreload = -0.25;
  private static final double kWheelSpeed2nd = 0.22;
  private static final double kWheelSpeed3rd = 0.45;
  private static final double kWheelSpeedHold = -0.15;
  private static final double kWheelSpeedPickup = -0.3;
  private static final double kWheelSpeedFirst = 0.10;
  private static final double kPreloadTime = 0.2;
  private static final double kLaunchTime = 0.5;

  private final CANSparkMax m_wheelsLeft = new CANSparkMax(kWheelsLeft, MotorType.kBrushless);
  private final CANSparkMax m_wheelsRight = new CANSparkMax(kWheelsRight, MotorType.kBrushless);

  private WheelLevel m_targetLevel = WheelLevel.First;
  private double m_wheelSpeed = 0;
  private static Intake m_intake;

  /** Creates a new Wheels. */
  public Wheels(Intake intake) {

    m_intake = intake;
    m_wheelsLeft.restoreFactoryDefaults();
    m_wheelsLeft.setIdleMode(IdleMode.kBrake);
    m_wheelsLeft.setInverted(true);

    m_wheelsRight.restoreFactoryDefaults();
    m_wheelsRight.setIdleMode(IdleMode.kBrake);
    m_wheelsRight.setInverted(false);

    m_wheelsLeft.burnFlash();
    m_wheelsRight.burnFlash();

    this.setDefaultCommand(this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_intake.getPosition() == m_intake.kInsideRad) m_wheelSpeed = kWheelSpeedHold;
    m_wheelsLeft.set(m_wheelSpeed);
    m_wheelsRight.set(m_wheelSpeed);

    // System.out.println(m_wheelSpeed + "    " + m_targetLevel);
  }

  /**
   * Set the target wheel speed in percent corresponding to a level
   *
   * @param level target level
   */
  private void setSpeedFor(WheelLevel wlevel) {
    switch (wlevel) {
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
      case Hold:
        m_wheelSpeed = kWheelSpeedHold;
        break;
      case First:
        m_wheelSpeed = kWheelSpeedFirst;
        break;
    }
  }

  /**
   * Hold the speed for a throw level
   *
   * @param level throwing level
   * @return forever command
   */
  public Command holdSpeed(WheelLevel wlevel) {
    return this.run(() -> this.setSpeedFor(wlevel));
  }

  /**
   * Stop the intake wheels
   *
   * @return instant command
   */
  public Command stop() {
    return this.runOnce(() -> this.setSpeedFor(WheelLevel.Stop));
  }

  public Command launchTo() {
    return Commands.sequence(
        this.holdSpeed(WheelLevel.Preload).withTimeout(kPreloadTime),
        this.setSpeedWithTarget().withTimeout(kLaunchTime),
        this.stop());
  }

  public Command setTargetLevel(WheelLevel level) {
    return this.runOnce(() -> m_targetLevel = level);
  }

  public Command setSpeedWithTarget() {
    return this.run(() -> this.setSpeedFor(m_targetLevel));
  }
}
