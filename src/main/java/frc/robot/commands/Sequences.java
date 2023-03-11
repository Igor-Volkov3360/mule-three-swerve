// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Level;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Position;
import frc.robot.subsystems.PivotArm;

/** Add your docs here. */
public class Sequences {

  public static Command PickConeFromFeeder(Elevator elevator, PivotArm pivotArm, Gripper gripper) {
    return Commands.parallel(elevator.extendTo(Level.Feeder), pivotArm.setTarget("up"));
    // gripper.setTarget("open").withTimeout(1.0));
  }

  public static Command SwitchToCone(Elevator elevator, Intake intake, Command switchMode) {
    return Commands.sequence(
        intake.setAngle(Position.Pickup),
        elevator.extendTo(Level.Second),
        intake.setAngle(Position.Retracted),
        elevator.extendTo(Level.Down),
        switchMode);
  }

  public static Command SwitchToCube(Elevator elevator, Intake intake, Command switchMode) {
    return Commands.sequence(
        elevator.extendTo(Level.Second),
        intake.setAngle(Position.Pickup),
        elevator.extendTo(Level.Down),
        intake.setAngle(Position.Retracted),
        switchMode);
  }
}
