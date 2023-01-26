// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem for wrapping communication with the vision co-processor */
public class Vision extends SubsystemBase {

  VisionMeasurement m_latestMeasure = null;

  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Get the latest available measurement from the vision system
   *
   * @return latest measurement or null
   */
  public VisionMeasurement getMeasurement() {
    return this.m_latestMeasure;
  }
}
