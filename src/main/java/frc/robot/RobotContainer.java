// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autonomous;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotArm;
// import frc.robot.subsystems.RGBControl;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Vision m_vision = new Vision();
  private final DriveTrain m_drive = new DriveTrain(m_vision);

  // private final RGBControl m_rgbPanel = new RGBControl();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();
  private final Spindexer m_spindexer = new Spindexer();
  private final PivotArm m_pivotArm = new PivotArm();
  private final Gripper m_gripper = new Gripper(m_pivotArm);

  private final double thirdLvl = 0.62;
  private final double secondLvl = 0.42;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Drive in robot relative velocities
    // Axis are inverted to follow North-West-Up (NWU) convention

    m_drive.setDefaultCommand(
        m_drive.driveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX(),
            false));

    // m_gripper.setDefaultCommand(m_gripper.openCommand());
    // Configure the trigger bindings

    m_gripper.setDefaultCommand(m_gripper.setTarget("open"));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().onTrue(m_elevator.extendTo(0.05));
    m_driverController.b().onTrue(m_elevator.down());

    m_driverController.povUp().onTrue(m_pivotArm.setTarget("up"));
    m_driverController.povDown().onTrue(m_pivotArm.setTarget("down"));
    m_driverController.povRight().onTrue(m_pivotArm.setTarget("cube"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autonomous.followTestTraj(m_drive);
  }

  /**
   * This command is used ot intake cubes
   *
   * @return the sequence that is used to intake game pieces
   */
  public Command IntakeOutSequenceCube() {
    return m_intake
        .setTarget("down")
        .andThen(
            m_intake
                .spin("cube")
                .alongWith(m_pivotArm.setTarget("cube").alongWith(m_spindexer.spin()))
                .alongWith(m_gripper.setTarget("cube")));
  }

  /**
   * This command is used ot intake cones
   *
   * @return the sequence that is used to intake game pieces
   */
  public Command IntakeOutSequenceCone() {
    return m_intake
        .setTarget("down")
        .andThen(
            m_intake
                .spin("cone")
                .alongWith(m_pivotArm.setTarget("up").alongWith(m_spindexer.spin())));
  }

  /**
   * This function retracts the intake
   *
   * @return the sequence that retracts the intake
   */
  public Command IntakeInSequence() {
    return m_pivotArm
        .setTarget("down")
        .alongWith(
            m_intake.setTarget("up").alongWith(m_spindexer.index()).andThen(m_intake.stop()));
  }

  public Command secondStageSequence() {
    return m_gripper
        .setTarget("cube")
        .withTimeout(0.25)
        .andThen(
            m_pivotArm
                .setTarget("up")
                .andThen(m_elevator.extendTo(secondLvl))
                .alongWith(m_gripper.setTarget("cube"))
                .withTimeout(1)
                .andThen(m_gripper.setTarget("open"))
                .withTimeout(2)
                .andThen(m_elevator.down()));
  }

  public Command thirdStageSequence() {
    return m_gripper
        .setTarget("cube")
        .withTimeout(0.25)
        .andThen(
            m_pivotArm
                .setTarget("up")
                .andThen(m_elevator.extendTo(thirdLvl))
                .alongWith(m_gripper.setTarget("cube"))
                .withTimeout(3)
                .andThen(m_gripper.setTarget("open"))
                .withTimeout(2)
                .andThen(m_elevator.down()));
  }
}
