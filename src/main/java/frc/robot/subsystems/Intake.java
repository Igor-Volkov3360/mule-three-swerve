// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Subsystem parameters
  private static final int kPivotLeft = 9;
  private static final int kPivotRight = 10;
  private static final int kRollerId = 17;

  private static final int kLeftEncoder = 8; // at CSM 10
  private static final int KRightEncoder = 7; // at CSM 9
  private static final double kTurnPerRotation = 0.25;
  private static final double kOffsetLeft = 0.17;
  private static final double kOffsetRight = 0.28;

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
  private static final double kRollerPercent = 0.7;

  // Member objects
  private final CANSparkMax m_pivotLeft = new CANSparkMax(kPivotLeft, MotorType.kBrushless);
  private final CANSparkMax m_pivotRight = new CANSparkMax(kPivotRight, MotorType.kBrushless);
  private final CANSparkMax m_roller = new CANSparkMax(kRollerId, MotorType.kBrushless);

  private final SparkMaxPIDController m_pidLeft = m_pivotLeft.getPIDController();
  private final AbsoluteEncoder m_encoderLeft = m_pivotLeft.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkMaxPIDController m_pidRight = m_pivotRight.getPIDController();
  private final AbsoluteEncoder m_encoderRight = m_pivotRight.getAbsoluteEncoder(Type.kDutyCycle);
  private final DutyCycleEncoder m_dutyEncoderLeft = new DutyCycleEncoder(kLeftEncoder);
  private final DutyCycleEncoder m_dutyEncoderRight = new DutyCycleEncoder(KRightEncoder);

  private final ShuffleboardTab m_intakeTab = Shuffleboard.getTab("intake");
  private final GenericEntry m_percentEntry = m_intakeTab.add("targetPercent", 0.0).getEntry();
  private final GenericEntry m_angleEntry = m_intakeTab.add("currentAngle", 0.0).getEntry();
  private final GenericEntry m_targetEntry = m_intakeTab.add("targetAngle", 0.0).getEntry();
  private final GenericEntry m_pidEntry = m_intakeTab.add("feedForward", 0.0).getEntry();

  // Process variables
  private double m_targetRad = kNeutralRad;
  private TrapezoidProfile m_profile = null;
  private double m_profileStart = -1.0;

  /** Creates a new Intake. */
  public Intake() {

    m_roller.restoreFactoryDefaults();
    m_roller.setInverted(true);
    m_roller.burnFlash();

    m_pivotLeft.restoreFactoryDefaults();
    m_pivotLeft.setIdleMode(IdleMode.kBrake);
    m_pivotLeft.setInverted(true);
    m_pivotLeft.enableVoltageCompensation(kNominalVolt);

    m_encoderLeft.setInverted(true);
    m_encoderLeft.setPositionConversionFactor(kNativeToRad);
    m_encoderLeft.setVelocityConversionFactor(kNativeToRad);
    m_encoderLeft.setZeroOffset(kZeroOffsetLeft);

    m_pidLeft.setFeedbackDevice(m_encoderLeft);
    m_pidLeft.setPositionPIDWrappingEnabled(true);
    m_pidLeft.setP(kP);
    m_pidLeft.setI(kI);
    m_pidLeft.setD(kD);
    m_pidLeft.setSmartMotionMaxVelocity(kAngVelRad, 0);
    m_pidLeft.setSmartMotionMaxAccel(kAngAccRed, 0);

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

    m_pivotLeft.burnFlash();
    m_pivotRight.burnFlash();
    // Stop intake by default

    m_dutyEncoderLeft.setDistancePerRotation(kTurnPerRotation);
    m_dutyEncoderLeft.setDistancePerRotation(kTurnPerRotation);
    m_dutyEncoderLeft.reset();
    m_dutyEncoderRight.reset();

    this.setDefaultCommand(this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Set target to current when robot is disabled to prevent sudden motion on enable
    if (DriverStation.isDisabled()) {
      m_targetRad = m_encoderLeft.getPosition();
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
    /*
    System.out.printf(
        "Pos L %.2f    Pos R %.2f    Pos* %.0f    FF %.3f    Cmd %.3f\n",
        Math.toDegrees(getEncoderLeft()),
        Math.toDegrees(getEncoderRight()),
        Math.toDegrees(m_targetRad),
        this.computeFeedForward(m_encoderLeft.getPosition(), kHorizontalPercentLeft),
        m_pivotLeft.getAppliedOutput());
        */
    System.out.println(getEncoderLeft() + "              " + getEncoderRight());
    System.out.println();
  }

  /**
   * Compute arbitrary feed-forward for current intake position
   *
   * @return feed-forward (percent)
   */
  private double computeFeedForward(double angle, double horizontalPercent) {
    return horizontalPercent * Math.cos(angle - Math.PI / 2.0);
  }

  /**
   * Check if the intake has reached its target angle
   *
   * @return intake is on target
   */
  private boolean onTarget() {
    return Math.abs(m_encoderLeft.getPosition() - m_targetRad) < kTargetTolRad
        && Math.abs(m_encoderRight.getPosition() - m_targetRad) < kTargetTolRad;
  }

  private boolean newOnTarget() {
    return Math.abs(rightEncoderToRad() - m_targetRad) < kTargetTolRad
        && Math.abs(leftEncoderToRad() - m_targetRad) < kTargetTolRad;
  }
  /**
   * Generate a trapezoidal motion profile to reach an angle from the current desired state
   *
   * @param targetRad target angle (rad)
   */
  private void generateProfile(double targetRad) {
    final double startRad =
        this.m_profile == null
            ? m_targetRad
            : this.m_profile.calculate(Timer.getFPGATimestamp() - m_profileStart).position;

    final double startVel =
        this.m_profile == null
            ? 0.0
            : this.m_profile.calculate(Timer.getFPGATimestamp() - m_profileStart).velocity;

    m_profile =
        new TrapezoidProfile(
            kProfileConstraints, new State(targetRad, 0), new State(startRad, startVel));
    m_profileStart = Timer.getFPGATimestamp();
    m_targetRad = targetRad;
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

  /**
   * Start the roller and extend the intake
   *
   * @return blocking command
   */
  public Command extend() {
    return this.operate(kRollerPercent, kExtendRad);
  }

  /**
   * Stop the roller and retract the intake
   *
   * @return blocking command
   */
  public Command retract() {
    return this.operate(0.0, kRetractRad);
  }

  /**
   * Stop the roller
   *
   * @return instant command
   */
  public Command stop() {
    return this.run(() -> m_roller.set(0.0));
  }

  public Command spin() {
    return this.run(() -> m_roller.set(kRollerPercent));
  }

  public Command changeTarget(double changeBy) {
    return this.runOnce(() -> m_targetRad += changeBy);
  }

  private double getEncoderLeft() {
    return m_dutyEncoderLeft.getAbsolutePosition() - kOffsetLeft;
  }

  private double getEncoderRight() {
    return 1 - m_dutyEncoderRight.getAbsolutePosition() - kOffsetRight;
  }

  private double leftEncoderToRad() {
    return getEncoderLeft();
  }

  private double rightEncoderToRad() {
    return getEncoderRight();
  }
}
