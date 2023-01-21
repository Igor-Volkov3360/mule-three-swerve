// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Wraps a vision robot pose measurement */
public class VisionMeasurement {

  /** Measurement timestamp (s) */
  public double m_timestamp;

  /** Field relative robot pose (m) */
  public Pose2d m_pose;
}
