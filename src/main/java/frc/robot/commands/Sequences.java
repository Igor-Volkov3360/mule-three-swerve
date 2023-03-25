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
import frc.robot.subsystems.Wheels.WheelLevel;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Wheels;

/** Add your docs here. */
public class Sequences {

  private static final double kPreloadTime = 0.2;
  private static final double kLaunchTime = 0.5;

  public static Command PickConeFromFeeder(Elevator elevator, PivotArm pivotArm, Gripper gripper) {
    return Commands.parallel(elevator.extendTo(Level.Feeder), pivotArm.setPivotState(true));
  }

  public static Command SwitchToCone(
      Elevator elevator, Intake intake, PivotArm pivotArm, Gripper gripper) {
    return Commands.sequence(
        intake.setAngle(Position.Pickup),
        pivotArm.setPivotState(true),
        new WaitCommand(kPreloadTime),
        elevator.extendTo(Level.Sequences),
        intake.setAngle(Position.Stored),
        new WaitCommand(0.3),
        elevator.extendTo(Level.Down));
  }

  public static Command SwitchToCube(Elevator elevator, Intake intake, PivotArm pivotArm, Wheels wheels) {
    return Commands.sequence(
        elevator.extendTo(Level.Sequences),
        new WaitCommand(0.3), // do NOT remove this delay, its perfect
        intake.setAngle(Position.Pickup),
        elevator.extendTo(Level.Down),
        pivotArm.setPivotState(false),
        intake.setAngle(Position.Retracted),
        wheels.stop());
  }

  public static Command scoreConeThird(Elevator elevator, PivotArm pivotArm, Gripper gripper) {
    return Commands.sequence(elevator.extendTo(Elevator.Level.Third), pivotArm.setPivotState(true));
  }

  public static Command scoreConeSecond(Elevator elevator, PivotArm pivotArm, Gripper gripper) {
    return Commands.sequence(
        elevator.extendTo(Elevator.Level.Second), pivotArm.setPivotState(true));
  }

  /**
   * Pickup a cube by lowering the intake and spinning until a cube is detected
   *
   * @return blocking command
   */
  public static Command pickup(Intake intake, Wheels wheels) {
    return Commands.sequence(
        intake.setAngle(Position.Pickup),
        wheels.holdSpeed(Wheels.WheelLevel.Pickup).until(intake::hasCube),
        intake.setAngle(Position.Retracted),
        wheels.setTargetLevel(Wheels.WheelLevel.Hold));
  }

  /**
   * This command launches a cube by raising the intake, preloding, and launching then stopping
   *
   * @return launch of a cube
   */
  public static Command launch(
      Intake intake, Wheels wheels, Wheels.WheelLevel level, Position position) {
    return Commands.sequence(
        intake.setAngle(position),
        wheels.holdSpeed(Wheels.WheelLevel.Preload).withTimeout(kPreloadTime),
        wheels.holdSpeed(level).withTimeout(kLaunchTime));
  }

  public static Command vomit(Intake intake, Wheels wheels) {
    return Commands.sequence(
        intake.setAngle(Position.Pickup).withTimeout(2),
        wheels.setTargetLevel(Wheels.WheelLevel.First),
        wheels.launchTo());
  }

  public static Command setTargetThirdIntake(Intake intake, Wheels wheels) {
    return Commands.sequence(
      intake.setAngle(Intake.Position.Launch),
      wheels.setTargetLevel(Wheels.WheelLevel.Hold), 
      wheels.setSpeedWithTarget(), 
      wheels.setTargetLevel(Wheels.WheelLevel.Third));
  }

  public static Command setTargetSecondIntake(Intake intake, Wheels wheels) {
    return Commands.sequence(
        intake.setAngle(Intake.Position.Launch),
        wheels.setTargetLevel(Wheels.WheelLevel.Hold), 
        wheels.setSpeedWithTarget(), 
        wheels.setTargetLevel(Wheels.WheelLevel.Second));
  }
}
