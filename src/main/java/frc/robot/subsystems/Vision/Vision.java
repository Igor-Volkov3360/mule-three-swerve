// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem for wrapping communication with the vision co-processor */
public class Vision extends SubsystemBase {

  VisionMeasurement m_latestMeasure = null;

  private DoubleArraySubscriber m_camPose =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getDoubleArrayTopic("position")
          .subscribe(new double[] {});

  private DoubleArraySubscriber m_camRotation =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getDoubleArrayTopic("rotation")
          .subscribe(new double[] {});

  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    final var position = m_camPose.get();
    final var rotation = m_camRotation.get();
    // System.out.println(position.length);

    if (position.length == 3 && rotation.length == 3) {
      var tagRotation = Rotation2d.fromDegrees(rotation[1]);
      var robotRotation = tagRotation;
      if (position[0] < 7.5) {
        robotRotation = tagRotation.rotateBy(Rotation2d.fromDegrees(180));
      }
      var measurement = new VisionMeasurement();

      measurement.m_timestamp = Timer.getFPGATimestamp();
      measurement.m_pose = new Pose2d(new Translation2d(position[0], position[1]), robotRotation);

      if (m_latestMeasure == null
          || (m_latestMeasure.m_pose.getX() != measurement.m_pose.getX()
              || m_latestMeasure.m_pose.getY() != measurement.m_pose.getY())) {
        m_latestMeasure = measurement;
      }
    }
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
