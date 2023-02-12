// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  // Subsystem parameters
  public static final int kLeadId = 14;
  public static final int kFollowId = 13;

  private static final double kNativeToMeter = 1.0 / 1024.0;
  private static final double kNominalVolt = 10.0;

  private static final double kNeutralMeter = 0.0;
  private static final double kGravityPercent = 0.0;

  private static final double kTargetTolMeter = 0.1;
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double kVelMeter = 1.0;
  private static final double kAccMeter = 2.0;

  // Member objects
  private final TalonSRX m_lead = new TalonSRX(kLeadId);
  private final TalonSRX m_follow = new TalonSRX(kFollowId);

  // Process variables
  private double m_targetMeter = kNeutralMeter;

  /** Creates a new Elevator. */
  public Elevator() {

    m_lead.configFactoryDefault();
    m_lead.configVoltageCompSaturation(kNominalVolt);
    m_lead.enableVoltageCompensation(true);

    m_lead.setSelectedSensorPosition(kNeutralMeter / kNativeToMeter);

    m_lead.config_kP(0, kP);
    m_lead.config_kI(0, kI);
    m_lead.config_kD(0, kD);

    m_lead.configMotionCruiseVelocity(kVelMeter / kNativeToMeter / 10.0);
    m_lead.configMotionAcceleration(kAccMeter / kNativeToMeter / 10.0);

    m_follow.configFactoryDefault();
    m_follow.configVoltageCompSaturation(kNominalVolt);
    m_follow.enableVoltageCompensation(true);
    m_follow.follow(m_lead);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Set target to current when robot is disabled to preven sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetMeter = m_lead.getSelectedSensorPosition() * kNativeToMeter;
    }

    // Set reference in periodic to allow arbitrary feed-forward
    m_lead.set(
        ControlMode.MotionMagic,
        m_targetMeter / kNativeToMeter,
        DemandType.ArbitraryFeedForward,
        kGravityPercent);
  }

  /**
   * Check wether the elevator reached its target extension
   *
   * @return elevator is at target
   */
  private boolean onTarget() {
    return Math.abs(m_lead.getSelectedSensorPosition() * kNativeToMeter - m_targetMeter)
        < kTargetTolMeter;
  }

  /**
   * Extend the elevator to a given length
   *
   * @param meters length (meter)
   * @return blocking command
   */
  public Command extendTo(double meters) {
    return this.run(() -> m_targetMeter = meters).until(this::onTarget);
  }
}
