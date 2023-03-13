// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Level;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Position;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Wheels;

/** Add your docs here. */
public class Sequences {

  public static Command PickConeFromFeeder(Elevator elevator, PivotArm pivotArm, Gripper gripper) {
    return Commands.parallel(elevator.extendTo(Level.Feeder), pivotArm.setTarget("up"));
  }

  public static Command SwitchToCone(Elevator elevator, Intake intake, PivotArm pivotArm) {
    return Commands.sequence(
        intake.setAngle(Position.Pickup),
        pivotArm.setTarget("up"),
        new WaitCommand(0.2),
        elevator.extendTo(Level.Sequences),
        intake.setAngle(Position.Stored),
        elevator.extendTo(Level.Down));
  }

  public static Command SwitchToCube(Elevator elevator, Intake intake, PivotArm pivotArm) {
    return Commands.sequence(
        elevator.extendTo(Level.Sequences),
        new WaitCommand(0.3), // do NOT remove this delay, its perfect
        intake.setAngle(Position.Pickup),
        elevator.extendTo(Level.Down),
        pivotArm.setTarget("down"));
  }

  public static Command scoreConeThird(Elevator elevator, PivotArm pivotArm, Gripper gripper) {
    return Commands.sequence(
        gripper.changeState(),
        elevator.extendTo(Elevator.Level.Third),
        pivotArm.setTarget("up"),
        gripper.changeState(),
        elevator.extendTo(Level.Down),
        pivotArm.setTarget("down"));
  }

  public static Command scoreConeSecond(Elevator elevator, PivotArm pivotArm, Gripper gripper) {
    return Commands.sequence(
        gripper.changeState(),
        elevator.extendTo(Elevator.Level.Second),
        pivotArm.setTarget("up"),
        gripper.changeState(),
        elevator.extendTo(Level.Down),
        pivotArm.setTarget("down"));
  }

  /**
   * Pickup a cube by lowering the intake and spinning until a cube is detected
   *
   * @return blocking command
   */
  public static Command pickup(Intake intake, Wheels wheels) {
    return Commands.sequence(
        intake.setAngle(Position.Pickup),
        wheels.holdSpeed(Wheels.Level.Pickup).until(intake::hasCube),
        intake.setAngle(Position.Retracted),
        wheels.holdSpeed(Wheels.Level.Hold).until(intake::noCube));
  }

  /**
   * This command launches a cube by raising the intake, preloding, and launching then stopping
   *
   * @return launch of a cube
   */
  public static Command launch(
      Intake intake, Wheels wheels, Wheels.Level level, Position position) {
    return Commands.sequence(
        intake.setAngle(position),
        wheels.holdSpeed(Wheels.Level.Preload).withTimeout(0.2),
        wheels.holdSpeed(level).withTimeout(0.5));
  }

  public static Command vomit(Intake intake, Wheels wheels) {
    return Commands.sequence(
        intake.setAngle(Position.Pickup).withTimeout(2),
        wheels.setTargetLevel(Wheels.Level.First),
        wheels.launchTo());
  }
}
