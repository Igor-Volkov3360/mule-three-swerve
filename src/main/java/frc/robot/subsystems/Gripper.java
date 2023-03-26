// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  // Subsystem parameters
  private static final int kGripperId = 15;

  private static final double kOpenPosition = 55;
  private static final double kClosePositionCone = 0;
  private static final double kCloseUpPositionCone = kClosePositionCone - 15;

  private static final double kp = 0.05;
  private double err = 0;
  private double cmd = 0;
  private static final int kMaxCurrent = 10;

  private double m_target = kClosePositionCone;
  private boolean m_open = false;
  private boolean m_isReseting = false;

  // Member objects
  private final CANSparkMax m_gripper = new CANSparkMax(kGripperId, MotorType.kBrushless);
  private PivotArm m_pivot;

  /** Creates a new Gripper. */
  public Gripper(PivotArm pivot) {

    m_gripper.restoreFactoryDefaults();
    m_gripper.setSmartCurrentLimit(kMaxCurrent);
    m_gripper.burnFlash();
    m_pivot = pivot;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!m_isReseting) {
      setTarget();
      err = m_target - m_gripper.getEncoder().getPosition();
      cmd = err * kp;
      m_gripper.set(MathUtil.clamp(cmd, -0.7, 0.7));
    } else {
      m_gripper.set(-0.4);
    }

    // System.out.println(m_open);
  }

  /**
   * This function is used to se the grab of the gripper
   *
   * @param position This parameter is used to dictate the opening
   * @return the command
   */
  public void setTarget() {
    if (m_open) m_target = kOpenPosition;
    else if (m_pivot.getTarget() == m_pivot.kDown) m_target = kClosePositionCone;
    else if (m_pivot.getTarget() == m_pivot.kUp) m_target = kCloseUpPositionCone;
  }

  public Command holdTarget() {
    return this.run(
        () -> {
          if (m_open) m_target = kOpenPosition;
          else if (m_pivot.getTarget() == m_pivot.kDown) m_target = kClosePositionCone;
          else if (m_pivot.getTarget() == m_pivot.kUp) m_target = kCloseUpPositionCone;
        });
  }

  public void reset(boolean resetOn) {
    m_isReseting = resetOn;
  }

  /**
   * Open and closes the gripper, true = open, false = close
   *
   * @return opening and closing of the gripper
   */
  public Command setGripperState(boolean state) {
    return this.runOnce(() -> m_open = state);
  }

  // public Command changeState() {
  //   Command tempCommand;
  //   if (m_open == true)
  //     tempCommand =
  //         this.runOnce(() -> m_open = false).andThen(this.runOnce(() -> this.setTarget()));
  //   else
  //     tempCommand = this.runOnce(() -> m_open = true).andThen(this.runOnce(() ->
  // this.setTarget()));
  //   return tempCommand;
  // }

  public Command defaultWinch() {
    return this.run(() -> this.reset(true))
        .withTimeout(1)
        .andThen(this.runOnce(() -> m_gripper.getEncoder().setPosition(0.0)))
        .andThen(this.runOnce(() -> this.reset(false)))
        .andThen(runOnce(() -> m_open = false));
  }

  public Command setTargetCurrent() {
    return this.runOnce(() -> m_target = m_gripper.getEncoder().getPosition());
  }

  public boolean gripperState() {
    return m_open;
  }
}
