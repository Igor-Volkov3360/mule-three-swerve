// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  public enum Level {
    Down,
    Feeder,
    Second,
    Third,
    Manual,
    DownManual,
    Sequences
  };

  // Subsystem parameters
  public static final int kLeadId = 14;
  public static final int kFollowId = 13;

  private static final double kNativeToMeter = 1.46 / 109.97;
  private static final double kNeutralMeter = 0.0;
  private static final double kDeadzone = 0.05;

  private static final double kP = 10.0;
  private static final double kVel = 1.2;
  private static final double kAcc = 2.0;

  private static final double kHeightDown = 0.0;
  private static final double kHeightFeeder = 1.0;
  private static final double kHeightSecond = 1.0;
  private static final double kHeightThird = 1.35;
  private static final double kManualHeight = 0.2;
  private static final double kDownManualHeight = -0.2;
  private static final double kSequencesHeight = 0.8;

  // Member objects
  private final CANSparkMax m_lead = new CANSparkMax(kLeadId, MotorType.kBrushless);
  private final CANSparkMax m_follow = new CANSparkMax(kFollowId, MotorType.kBrushless);
  private final DigitalInput m_limitSwitch = new DigitalInput(9);
  private final RelativeEncoder m_encoder = m_lead.getEncoder();
  private final ProfiledPIDController m_pid =
      new ProfiledPIDController(kP, 0.0, 0.0, new Constraints(kVel, kAcc));

  // Process variables
  private double m_targetMeter = kNeutralMeter;
  private Level m_targetLevel = Level.Down;

  /** Creates a new Elevator. */
  public Elevator() {

    m_lead.restoreFactoryDefaults();
    m_follow.restoreFactoryDefaults();

    m_lead.setIdleMode(IdleMode.kCoast);
    m_lead.setInverted(true);

    m_encoder.setPosition(kNeutralMeter);
    m_encoder.setPositionConversionFactor(kNativeToMeter);
    m_encoder.setVelocityConversionFactor(kNativeToMeter);

    m_follow.setIdleMode(IdleMode.kCoast);
    m_follow.follow(m_lead, true);

    m_lead.burnFlash();
    m_follow.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Reset encoder when bottom is reached
    if (isDown()) {
      m_encoder.setPosition(kHeightDown);
    }

    // Set target to current when robot is disabled to preven sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetMeter = m_encoder.getPosition();
      m_pid.reset(m_targetMeter);
    }
    m_lead.set(motorSpeed());

    /*
    System.out.printf(
        "Elevator height = %4.2f m\t target = %4.2f\t percent = %4.2f\n",
        m_encoder.getPosition(), m_pid.getSetpoint().position, m_lead.getAppliedOutput());

    */
  }

  /**
   * Set the target height to reach a given level
   *
   * @param level target level
   */
  private void setHeightFor(Level level) {
    switch (level) {
      case Down:
        m_targetMeter = kHeightDown;
        break;
      case Feeder:
        m_targetMeter = kHeightFeeder;
        break;
      case Second:
        m_targetMeter = kHeightSecond;
        break;
      case Third:
        m_targetMeter = kHeightThird;
        break;
      case Manual:
        m_targetMeter = kManualHeight;
        break;
      case DownManual:
        m_targetMeter = kDownManualHeight;
        break;
      case Sequences:
        m_targetMeter = kSequencesHeight;
    }
  }

  /**
   * Extend the elevator to a given length
   *
   * @param meters length (meter)
   * @return blocking command
   */
  public Command extendTo(Level level) {
    return new SequentialCommandGroup(
        this.runOnce(() -> m_pid.reset(m_encoder.getPosition())),
        this.run(() -> this.setHeightFor(level)).until(this::onTarget));
  }

  public Command stop() {
    return this.runOnce(
        () -> {
          m_lead.set(0.0);
        });
  }

  private double motorSpeed() {
    return m_pid.calculate(m_encoder.getPosition(), m_targetMeter);
  }

  public boolean onTarget() {
    return Math.abs(m_targetMeter - m_encoder.getPosition()) < kDeadzone;
  }

  public boolean isDown() {
    return !m_limitSwitch.get();
  }

  public Command extend() {

    return new SequentialCommandGroup(
        this.runOnce(() -> m_pid.reset(m_encoder.getPosition())),
        this.run(() -> this.setHeightFor(m_targetLevel)).until(this::onTarget));
  }

  public Command setTargetLevel(Level level) {
    return this.runOnce(() -> m_targetLevel = level);
  }
}
