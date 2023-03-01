// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  // Subsystem parameters
  public static final int kLeadId = 14;
  public static final int kFollowId = 13;

  private static final double kNativeToMeter = 1.51 / 13200;
  private static final double kNeutralMeter = 0.0;

  // Member objects
  private final CANSparkMax m_lead = new CANSparkMax(kLeadId, MotorType.kBrushless);
  private final CANSparkMax m_follow = new CANSparkMax(kFollowId, MotorType.kBrushless);

  // Process variables
  private double m_targetMeter = kNeutralMeter;

  /** Creates a new Elevator. */
  public Elevator() {
    m_lead.setIdleMode(IdleMode.kCoast);
    m_follow.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Set target to current when robot is disabled to preven sudden motion on enable

    if (DriverStation.isDisabled()) {
      m_targetMeter = getEncoder() * kNativeToMeter;
    }

    m_lead.set(normalizeValue());
    m_follow.set(normalizeValue());
  }

  /**
   * Extend the elevator to a given length
   *
   * @param meters length (meter)
   * @return blocking command
   */
  public Command extendTo(double meters) {
    return this.runOnce(() -> m_targetMeter = meters);
  }

  public Command down() {
    return this.runOnce(() -> m_targetMeter = 0.0);
  }

  private double motorSpeed() {
    return getEncoder() * kNativeToMeter - m_targetMeter;
  }

  private double getEncoder() {
    return m_lead.getAlternateEncoder(4096).getPosition();
    // countsPerRev was 4096 (aka for CTRE SRX)
  }

  private double normalizeValue() {
    return -((motorSpeed() - 0) / (1 - 0));
  }
}
