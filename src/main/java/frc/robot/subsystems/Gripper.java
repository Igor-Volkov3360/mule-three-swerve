// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Gripper extends SubsystemBase {

  // Subsystem parameters
  private final int kGripperId = 0;
  private final double kClosePercent = 0.2;
  private final double kOpenPercent = 0.0;
  private final double kTransitSeconds = 0.5;

  // Member objects
  private final CANSparkMax m_gripper = new CANSparkMax(kGripperId, MotorType.kBrushless);

  // Process variables
  private double m_outputPercent = kOpenPercent;

  /** Creates a new Gripper. */
  public Gripper() {

    m_gripper.restoreFactoryDefaults();
    m_gripper.burnFlash();
  }

  /**
   * Open the gripper
   *
   * @return blocking command
   */
  public Command open() {

    return this.runOnce(() -> m_outputPercent = kOpenPercent)
        .andThen(new WaitCommand(kTransitSeconds));
  }

  /**
   * Close the gripper
   *
   * @return blocking command
   */
  public Command close() {

    return this.runOnce(() -> m_outputPercent = kClosePercent)
        .andThen(new WaitCommand(kTransitSeconds));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_gripper.set(m_outputPercent);
  }
}
