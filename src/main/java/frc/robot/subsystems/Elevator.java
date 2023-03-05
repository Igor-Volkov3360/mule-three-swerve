// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  // Subsystem parameters
  public static final int kLeadId = 14;
  public static final int kFollowId = 13;

  private static final double kNativeToMeter = 1.46 / 109.97;
  private static final double kNeutralMeter = 0.0;
  private static final double deadzone = 0.05;
  private static final double kDown = -0.01;

  private static final double kMultiplier = 2.5;

  // Member objects
  private final CANSparkMax m_lead = new CANSparkMax(kLeadId, MotorType.kBrushless);
  private final CANSparkMax m_follow = new CANSparkMax(kFollowId, MotorType.kBrushless);
  private final DigitalInput m_limitSwitch = new DigitalInput(9);

  // Process variables
  private double m_targetMeter = kNeutralMeter;

  /** Creates a new Elevator. */
  public Elevator() {
    m_lead.setIdleMode(IdleMode.kCoast);
    m_follow.setIdleMode(IdleMode.kCoast);
    this.setDefaultCommand(this.down());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    limitSwitch();

    if (limitSwitch()) m_lead.getEncoder().setPosition(0);

    // Set target to current when robot is disabled to preven sudden motion on enable

    if (DriverStation.isDisabled()) {
      m_targetMeter = getEncoder() * kNativeToMeter;
    }

    m_lead.set(normalizeValue() * kMultiplier);
    m_follow.set(normalizeValue() * kMultiplier);

    if (isOnTarget()) m_lead.set(0);

    // System.out.println(isOnTarget() + "         " + getEncoder());
  }

  /**
   * Extend the elevator to a given length
   *
   * @param meters length (meter)
   * @return blocking command
   */
  public Command extendTo(double meters) {
    return this.run(() -> m_targetMeter = meters);
  }

  public Command down() {
    return this.runOnce(() -> m_targetMeter = kDown).until(this::limitSwitch).andThen(stop());
  }

  public Command stop() {
    return this.runOnce(
        () -> {
          m_lead.set(0);
          m_follow.set(0);
        });
  }

  private double motorSpeed() {
    return getEncoder() * kNativeToMeter - m_targetMeter;
  }

  private double getEncoder() {
    return m_lead.getEncoder().getPosition();
  }

  private double normalizeValue() {
    return -((motorSpeed() - 0) / (1 - 0));
  }

  public boolean isOnTarget() {
    return getEncoder() < deadzone && getEncoder() > -deadzone;
  }

  public boolean limitSwitch() {
    return !m_limitSwitch.get();
  }
}
