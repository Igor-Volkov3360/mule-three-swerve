// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Subsystem parameters
  private static final int kPivotLeft = 9;
  private static final int kPivotRight = 10;

  private static final double kNativeToRad = 2.0 * Math.PI;
  private static final double kNominalVolt = 10.0;

  private static final double kNeutralRad = Math.PI / 2.0;
  private static final double kEncOff = 3.0 * Math.PI / 2.0;
  private static final double kZeroOffsetLeft = 0.63 + kEncOff;
  private static final double kZeroOffsetRight = 0.0 + kEncOff;
  private static final double kHorizontalPercentLeft = 0.025;
  private static final double kHorizontalPercentRight = 0.035;

  private static final double kTargetTolRad = Math.toRadians(5.0);
  private static final double kP = 0.8; // have to continue tweaking
  private static final double kI = 0.0;
  private static final double kD = 0.000;

  private static final double kAngVelRad = Math.toRadians(180.0);
  private static final double kAngAccRed = Math.toRadians(60.0);
  private static final TrapezoidProfile.Constraints kProfileConstraints =
      new TrapezoidProfile.Constraints(kAngVelRad, kAngAccRed);

  private static final double kRetractRad = Math.toRadians(90.0 + 72.0);
  private static final double kExtendRad = Math.toRadians(90.0);
  private static final double kRollerPercent = 0.5;

  // Member objects
  private final CANSparkMax m_pivotLeft = new CANSparkMax(kPivotLeft, MotorType.kBrushless);
  private final CANSparkMax m_pivotRight = new CANSparkMax(kPivotRight, MotorType.kBrushless);
  private final DutyCycleEncoder m_dutyEncoderLeft = new DutyCycleEncoder(kLeftEncoder);
  private final DutyCycleEncoder m_dutyEncoderRight = new DutyCycleEncoder(KRightEncoder);

  private static PowerDistribution m_pdp = new PowerDistribution(20, ModuleType.kRev);

  /** Creates a new Intake. */
  public Intake() {

    m_roller.restoreFactoryDefaults();
    m_roller.setInverted(true);
    m_roller.burnFlash();

    m_pivotLeft.restoreFactoryDefaults();
    m_pivotLeft.setInverted(true);
    m_pivotLeft.enableVoltageCompensation(kNominalVolt);

    m_encoderLeft.setInverted(true);
    m_encoderLeft.setPositionConversionFactor(kNativeToRad);
    m_encoderLeft.setVelocityConversionFactor(kNativeToRad);
    m_encoderLeft.setZeroOffset(kZeroOffsetLeft);

    m_pidLeft.setFeedbackDevice(SparkMaxRelativeEncoder.Type.kQuadrature);
    m_pidLeft.setPositionPIDWrappingEnabled(true);
    m_pidLeft.setP(kP);
    m_pidLeft.setD(kD);
    m_pidLeft.setI(kI);
    m_pidLeft.setSmartMotionMaxVelocity(kAngVelRad, 0);
    m_pidLeft.setSmartMotionMaxAccel(kAngAccRed, 0);

    m_pivotLeft.burnFlash();

    m_pivotRight.restoreFactoryDefaults();
    m_pivotRight.setIdleMode(IdleMode.kBrake);
    m_pivotRight.enableVoltageCompensation(kNominalVolt);

    m_encoderRight.setPositionConversionFactor(kNativeToRad);
    m_encoderRight.setVelocityConversionFactor(kNativeToRad);
    m_encoderRight.setZeroOffset(kZeroOffsetRight);

    m_pidRight.setFeedbackDevice(m_encoderRight);
    m_pidRight.setPositionPIDWrappingEnabled(true);
    m_pidRight.setP(kP);
    m_pidRight.setD(kD);
    m_pidRight.setI(kI);
    m_pidRight.setSmartMotionMaxVelocity(kAngVelRad, 0);
    m_pidRight.setSmartMotionMaxAccel(kAngAccRed, 0);

    m_pivotRight.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    coneLift();

    // Set target to current when robot is disabled to prevent sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetLeft = (getLeftEncoder() + getRightEncoder()) / 2;
      m_targetRight = (getLeftEncoder() + getRightEncoder()) / 2;
    }

    // Clear motion profile if target is reached
    if (this.onTarget()) {
      this.m_profile = null;
    }

    // Set reference in periodic to allow for arbitrary PID computation
    if (m_profile != null) {
      // Pivot is tracking a velocity profile
      final var profTime = Timer.getFPGATimestamp() - m_profileStart;
      final var profTarget = m_profile.calculate(profTime);

      m_pidLeft.setReference(
          profTarget.position,
          ControlType.kPosition,
          0,
          this.computeFeedForward(profTarget.position, kHorizontalPercentLeft),
          ArbFFUnits.kPercentOut);
      m_pidRight.setReference(
          profTarget.position,
          ControlType.kPosition,
          0,
          this.computeFeedForward(profTarget.position, kHorizontalPercentRight),
          ArbFFUnits.kPercentOut);
    } else {
      // Pivot has reached its target thus holding its position
      m_pidLeft.setReference(
          m_targetRad,
          ControlType.kPosition,
          0,
          this.computeFeedForward(m_targetRad, kHorizontalPercentLeft),
          ArbFFUnits.kPercentOut);
      m_pidRight.setReference(
          m_targetRad,
          ControlType.kPosition,
          0,
          this.computeFeedForward(m_targetRad, kHorizontalPercentRight),
          ArbFFUnits.kPercentOut);
    }

    m_angleEntry.setDouble(Math.toDegrees(m_encoderLeft.getPosition()));
    m_percentEntry.setDouble(m_pivotLeft.getAppliedOutput());
    m_targetEntry.setDouble(Math.toDegrees(m_targetRad));
    m_pidEntry.setDouble(
        this.computeFeedForward(m_encoderLeft.getPosition(), kHorizontalPercentLeft));

    System.out.printf(
        "Pos L %.2f    Pos R %.2f    Pos* %.0f    FF %.3f    Cmd %.3f\n",
        Math.toDegrees(m_encoderLeft.getPosition()),
        Math.toDegrees(m_encoderRight.getPosition()),
        Math.toDegrees(m_targetRad),
        this.computeFeedForward(m_encoderLeft.getPosition(), kHorizontalPercentLeft),
        m_pivotLeft.getAppliedOutput());
  }

  /**
   * Gets the value of the left encoder between 0 and 0.18
   *
   * @return A value of 0 when the intake is down
   */
  private double computeFeedForward(double angle, double horizontalPercent) {
    return horizontalPercent * Math.cos(angle - Math.PI / 2.0);
  }

  /**
   * Gets the value of the right encoder between 0 and 0.18
   *
   * @return A value of 0 when the intake is down
   */
  private double getRightEncoder() {
    return m_dutyEncoderRight.getAbsolutePosition() - kOffsetRight;
  }

  private boolean isLeftInFastRange() {
    return getLeftEncoder() > kLowwerSoftLimit && getLeftEncoder() < kUpperSoftLimit;
  }

  /**
   * Set the roller speed and pivot to a given angle
   *
   * @param rollerPercent roller speed (percent)
   * @param pivotRad angle above horizontal (rad)
   * @return blocking command
   */
  public Command operate(double rollerPercent, double pivotRad) {
    return this.runOnce(() -> this.generateProfile(pivotRad))
        .andThen(this.run(() -> m_roller.set(rollerPercent)).until(this::onTarget));
  }

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

  public double motorSpeedLeft() {
    return m_targetLeft - getLeftEncoder();
  }

  public Command spin() {
    return this.run(() -> m_roller.set(kRollerPercent));
  }

  private void coneLift() {
    if (m_pdp.getCurrent(7) < kCurrentThreshold) setTarget("cone");
  }
}
