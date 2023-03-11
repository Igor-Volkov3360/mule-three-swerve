// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/** Robot autonomous sequences */
public class Autonomous {

  /** Follow a simple path from the robot start location */
  public static Command followTestTraj(DriveTrain driveTrain) {

    return driveTrain.followPathCommand(PathPlanner.loadPath("marker", 4.0, 4.0), true, true);
  }
}
