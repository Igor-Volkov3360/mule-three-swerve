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
  private static final double kClosePercent = 0.2;
  private static final double kOpenPercent = 0.0;
  private static final double kTransitSeconds = 0.5;

  private static final double kCloseCurrentCone = 1;
  private static final double kCloseCurrentCube = 1;
  private static final double kOpenPosition = 0;
  private static final double kClosePosition = 0;

  public static final double kCurrentThreshold = 5.0;

  // Member objects
  private final CANSparkMax m_gripper = new CANSparkMax(kGripperId, MotorType.kBrushless);

  /** Creates a new Gripper. */
  public Gripper() {

    m_gripper.restoreFactoryDefaults();
    m_gripper.burnFlash();

    // Default to open as gripper will open when robot is disabled
    this.setDefaultCommand(this.open());
  }

  /**
   * Open the gripper
   *
   * @return blocking command
   */
  public Command open() {

    return this.run(() -> m_gripper.set(kOpenPercent));
  }

  /**
   * Close the gripper
   *
   * @return blocking command
   */
  public Command close(String gamePiece) {

    return this.run(
        () -> {
          if (!reachCurrent()) m_gripper.set(kClosePercent);
        });
    // 1. function return boolean check if current too high
    // 2.close until previous is true
  }

  private Boolean reachCurrent() {
    return kCurrentThreshold < m_gripper.getAppliedOutput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(m_gripper.getAppliedOutput());
  }
}
