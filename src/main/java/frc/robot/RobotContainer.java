// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autonomous;
import frc.robot.commands.Sequences;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Level;
import frc.robot.subsystems.Intake.Position;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.RGBControl;
import frc.robot.subsystems.Vision.Vision;
import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private enum RobotMode {
    Cube,
    Cone
  };

  // The robot's subsystems and commands are defined here...
  private final Vision m_vision = new Vision();
  private final DriveTrain m_drive = new DriveTrain(m_vision);

  // private final RGBControl m_rgbPanel = new RGBControl();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();
  private final PivotArm m_pivotArm = new PivotArm();
  private final Gripper m_gripper = new Gripper(m_pivotArm);
  private final RGBControl m_rgbPanel = new RGBControl();

  private static final PathConstraints constraints = new PathConstraints(4.0, 8.0);
  public static final PathPlannerTrajectory path = PathPlanner.loadPath("marker", constraints);

  public static HashMap<String, Command> eventMap = new HashMap<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_coDriverController = new CommandXboxController(1);

  // Process variables
  private RobotMode m_currentMode = RobotMode.Cube;

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
    m_rgbPanel.setDefaultCommand(m_rgbPanel.teamCommand());
    // m_pivotArm.setDefaultCommand(m_pivotArm.setZero());
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

    m_driverController.a().onTrue(m_intake.pickup());
    m_driverController.b().onTrue(m_intake.setAngle(Position.Retracted));
    m_driverController.x().onTrue(m_intake.launch(Level.First, Position.Pickup));
    m_driverController.y().onTrue(m_elevator.extendTo(Elevator.Level.Feeder));
    m_driverController.leftBumper().whileTrue(visionMode());
    // m_driverController.start().onTrue(enableBodyClimb());
    // m_driverController.rightTrigger().onTrue(bodyClimb());
    // m_driverController.leftTrigger().onTrue(bodyClimb());

    // m_coDriverController.a().onTrue(m_intake.launch(null, null));

    // pick a cone from the feeder station
    m_coDriverController
        .a()
        .onTrue(
            Sequences.PickConeFromFeeder(m_elevator, m_pivotArm, m_gripper)
                .unless(this::inCubeMode));

    m_coDriverController.b().toggleOnTrue(m_gripper.setTarget("close"));
    m_coDriverController.x().onTrue(m_intake.setAngle(Intake.Position.Launch));
    m_coDriverController.y().onTrue(m_intake.launch(Intake.Level.Third, Intake.Position.Launch));
    m_coDriverController.leftBumper().onTrue(m_elevator.extendTo(Elevator.Level.Down));

    // toggle robot modes
    m_coDriverController
        .start()
        .onTrue(
            Sequences.SwitchToCube(m_elevator, m_intake, this.setMode(RobotMode.Cube))
                .unless(this::inCubeMode));
    m_coDriverController
        .back()
        .onTrue(
            Sequences.SwitchToCone(
                m_elevator, m_intake, this.setMode(RobotMode.Cone).unless(this::inConeMode)));

    // m_coDriverController.povLeft().onTrue(navigateLeft());
    // m_coDriverController.povRight().onTrue(navigateRight());
    m_coDriverController.leftTrigger().whileTrue(m_elevator.extendTo(Elevator.Level.Manual));
    m_coDriverController.rightTrigger().whileTrue(m_elevator.extendTo(Elevator.Level.DownManual));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return this.runAuto();
  }

  /**
   * Indicates if the current mode is cube
   *
   * @return robot is in cube mode
   */
  public boolean inCubeMode() {
    return this.m_currentMode == RobotMode.Cube;
  }

  /**
   * Indicates if the current mode is cone
   *
   * @return robot is in cone mode
   */
  public boolean inConeMode() {
    return this.m_currentMode == RobotMode.Cone;
  }

  /**
   * Sets the current mode
   *
   * @param newMode new robot mode
   * @return instant command
   */
  public Command setMode(RobotMode newMode) {
    return new InstantCommand(() -> m_currentMode = newMode);
  }

  /**
   * This command is used ot intake cubes
   *
   * @return the sequence that is used to intake game pieces
   */
  public Command IntakeOutSequenceCube() {
    return m_intake
        .setAngle(Position.Pickup)
        .andThen(m_pivotArm.setTarget("cube").alongWith(m_gripper.setTarget("cube")));
  }

  /**
   * This command is used ot intake cones
   *
   * @return the sequence that is used to intake game pieces
   */
  public Command IntakeOutSequenceCone() {
    return m_intake.setAngle(Position.Pickup).andThen(m_pivotArm.setTarget("up"));
  }

  /**
   * This function retracts the intake
   *
   * @return the sequence that retracts the intake
   */
  public Command IntakeInSequence() {
    return m_pivotArm.setTarget("down").alongWith(m_intake.setAngle(Position.Retracted));
  }

  public Command secondStageSequence() {
    return m_gripper
        .setTarget("cube")
        .withTimeout(0.25)
        .andThen(
            m_pivotArm
                .setTarget("up")
                .until(m_pivotArm::isOnTarget)
                .andThen(m_elevator.extendTo(Elevator.Level.Second))
                .until(m_elevator::onTarget)
                .alongWith(m_gripper.setTarget("cube"))
                .withTimeout(1)
                .andThen(m_gripper.setTarget("open"))
                .withTimeout(2)
                .andThen(m_elevator.extendTo(Elevator.Level.Down))
                .until(m_elevator::onTarget));
  }

  public Command thirdStageSequence() {
    return m_gripper
        .setTarget("cube")
        .withTimeout(0.25)
        .andThen(
            m_pivotArm
                .setTarget("up")
                .andThen(m_elevator.extendTo(Elevator.Level.Third))
                .alongWith(m_gripper.setTarget("cube"))
                .withTimeout(3)
                .andThen(m_gripper.setTarget("open"))
                .withTimeout(2)
                .andThen(m_elevator.extendTo(Elevator.Level.Down)));
  }

  public Command runAuto() {

    eventMap.put("yez", m_pivotArm.setTarget("up"));

    // eventMap.put("lift", m_elevator.extendTo(0.8));

    Command auto =
        new FollowPathWithEvents(Autonomous.followTestTraj(m_drive), path.getMarkers(), eventMap);

    return auto;
  }

  public Command visionMode() {
    return null;
  }
}
