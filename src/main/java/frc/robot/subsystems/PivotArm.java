// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotArm extends SubsystemBase {

  // Subsystem parameters
  private static final int kPivotId = 16;
  private static final double kNativeToRad = 1.0;
  private static final double kNominalVolt = 10.0;
  private static final double kHorizontalPercent = 0.02;
  private static final double kNeutralRad = 0.0;

  public final double kUp = 1.9; // when the gripper is PARRALLEL to the ground
  public final double kDown = 0.0; // when the gripper is PERPENDICULAR to the ground
  private static final double kCube = 0.8;
  private double m_target = kNeutralRad;
  private static final double kP = 0.15;
  private static final double kMaxVel = 1.8;
  private static final double kMaxAcc = 0.5 * kMaxVel;
  private static final double deadzone = 0.01;

  // Member objects
  private final CANSparkMax m_pivot = new CANSparkMax(kPivotId, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_pivot.getEncoder();
  private final SparkMaxPIDController m_pid = m_pivot.getPIDController();

  /** Creates a new PivotArm. */
  public PivotArm() {

    m_pivot.restoreFactoryDefaults();
    m_pivot.enableVoltageCompensation(kNominalVolt);

    m_encoder.setPositionConversionFactor(kNativeToRad);
    m_encoder.setVelocityConversionFactor(kNativeToRad);
    m_encoder.setPosition(kNeutralRad);

    m_pid.setSmartMotionMaxVelocity(kMaxVel, 0);
    m_pid.setSmartMotionMaxAccel(kMaxAcc, 0);
    m_pid.setP(kP);

    m_pivot.burnFlash();
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled()) {
      m_target = m_encoder.getPosition();
    }

    final var ff = kHorizontalPercent * Math.sin(m_encoder.getPosition());
    m_pid.setReference(m_target, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
  }

  /**
   * This function sets the desired angle for the pivot arm
   *
   * @param position The desired position for the pivot arm, "up", "down", "cube"
   * @return Desired angle for the pivot arm
   */
  public Command setTarget(String position) {
    return this.runOnce(
        () -> {
          if (position == "up") m_target = kUp;
          else if (position == "down") m_target = kDown;
          else if (position == "cube") m_target = kCube;
        });
  }
  // mini neo = 110rpm at 100%
  // neo 567 at 100%
  public Command compensate(double position, double speed) {
    return setTarget("down");
  }

  public double getTarget() {
    return m_target;
  }

  public boolean isOnTarget() {
    return m_encoder.getPosition() > deadzone + m_target
        && m_encoder.getPosition() < m_target - deadzone;
  }
}
