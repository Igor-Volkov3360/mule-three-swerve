// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  // Subsystem parameters
  private static final int kGripperId = 15;

  private static final double kOpenPosition = 0;
  private static final double kClosePositionCone = -30;
  private static final double kCloseUpPositionCone = kClosePositionCone - 30;

  private static final double kMultiplier = 0.1;

  private double m_target = kOpenPosition;
  private boolean m_open = true;

  // private double[] atRightPose = {0.0, 0.0, 0.0};

  // Member objects
  private final CANSparkMax m_gripper = new CANSparkMax(kGripperId, MotorType.kBrushless);
  private PivotArm m_pivot;

  /** Creates a new Gripper. */
  public Gripper(PivotArm pivot) {

    m_gripper.restoreFactoryDefaults();
    m_gripper.burnFlash();
    m_pivot = pivot;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_gripper.set(motorSpeed() * kMultiplier);
    System.out.println(m_target + "       " + m_open);
  }

  public Command stop() {
    return this.run(() -> m_gripper.set(0));
  }

  /**
   * This function is used to se the grab of the gripper
   *
   * @param position This parameter is used to dictate the opening
   * @return the command
   */
  public Command setTarget() {
    return this.runOnce(
        () -> {
          if (m_open) m_target = kOpenPosition;
          else if (m_pivot.getTarget() == m_pivot.kDown && !m_open) m_target = kClosePositionCone;
          else if (m_pivot.getTarget() == m_pivot.kUp && !m_open) m_target = kCloseUpPositionCone;
        });
  }

  private double motorSpeed() {
    return (m_target - m_gripper.getEncoder().getPosition()) / 2;
  }

  public Command manualWind() {
    return this.run(() -> m_gripper.set(-0.1));
  }

  /**
   * Open and closes the gripper, true = open, false = close
   *
   * @return opening and closing of the gripper
   */
  public Command changeState() {
    return this.runOnce(() -> m_open = !m_open).andThen(this.setTarget());
  }
}
