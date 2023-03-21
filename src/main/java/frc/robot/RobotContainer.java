// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Sequences;
import frc.robot.subsystems.BuddyClimb;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Level;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Position;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.RGBControl;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Wheels;
import frc.robot.subsystems.Wheels.WheelLevel;
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

  public boolean povUpPressed = false;
  public boolean povDownPressed = false;
  public boolean joystickInversion = false;

  // The robot's subsystems and commands are defined here...
  private final Vision m_vision = new Vision();
  private final DriveTrain m_drive = new DriveTrain(m_vision);
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();
  private final PivotArm m_pivotArm = new PivotArm();
  private final Gripper m_gripper = new Gripper(m_pivotArm);
  private final RGBControl m_rgbPanel = new RGBControl();
  private final Wheels m_wheels = new Wheels(m_intake);
  private final BuddyClimb m_buddyClimb = new BuddyClimb(m_intake, m_wheels, m_elevator);

  private static final PathConstraints constraints = new PathConstraints(4.0, 8.0);
  // different trajectories

  PathPlannerTrajectory line = PathPlanner.loadPath("line", constraints);

  PathPlannerTrajectory pathScoreCone2CubesLeft =
      PathPlanner.loadPath("begin with cone 2 cubes left", constraints);

  PathPlannerTrajectory pathScoreConeShootCubeBalanceLeft =
      PathPlanner.loadPath("cone cube balance left", constraints);

  PathPlannerTrajectory pathScoreConeShootCubeBalanceRight =
      PathPlanner.loadPath("cone cube balance right", constraints);

  PathPlannerTrajectory pathScoreCone2CubesRight =
      PathPlanner.loadPath("begin with cone 2 cubes right", constraints);

  PathPlannerTrajectory pathBalance = PathPlanner.loadPath("balance", constraints);

  PathPlannerTrajectory pathConeBalance = PathPlanner.loadPath("cone balance", constraints);

  PathPlannerTrajectory pathCubeBalance = PathPlanner.loadPath("cube balance", constraints);

  PathPlannerTrajectory pathCube = PathPlanner.loadPath("cube", constraints);

  PathPlannerTrajectory pathSingleCube = PathPlanner.loadPath("single cube", constraints);

  PathPlannerTrajectory path2Cubes = PathPlanner.loadPath("2 cubes", constraints);

  public static HashMap<String, Command> eventMap = new HashMap<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private static final CommandXboxController m_coDriverController = new CommandXboxController(1);

  // Process variables
  private RobotMode m_currentMode = RobotMode.Cone;

  SendableChooser<Command> m_chooser;
  ComplexWidget chooserList;
  ComplexWidget camera;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // creates sendable chooser
    m_chooser = new SendableChooser<>();

    // create options for auto mode
    m_chooser.setDefaultOption("shoot cube dont move", this.shootCube());
    m_chooser.addOption("stop", this.stop());
    m_chooser.addOption("line", m_drive.followPathCommand(line, true, true));
    m_chooser.addOption("balance", this.runPathBalance());
    chooserList =
        Shuffleboard.getTab("auto").add(m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    camera = Shuffleboard.getTab("vision").add(CameraServer.startAutomaticCapture());

    // Drive in robot relative velocities
    m_drive.setDefaultCommand(
        m_drive.driveCommand(
            () -> {
              final var driverY = m_driverController.getLeftY();
              return DriverStation.getAlliance() == Alliance.Blue ? -driverY : driverY;
            },
            () -> {
              final var driverX = m_driverController.getLeftX();
              return DriverStation.getAlliance() == Alliance.Blue ? -driverX : driverX;
            },
            () -> -m_driverController.getRightX(),
            true));
    // Configure the trigger bindings

    m_rgbPanel.setDefaultCommand(m_rgbPanel.teamCommand());

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

    // pick up coob
    m_driverController
        .a()
        .onTrue(
            Sequences.pickup(m_intake, m_wheels)
                .unless(this::inConeMode)
                .alongWith(m_rgbPanel.purpleCommand()));

    // retract intake
    m_driverController.b().onTrue(m_intake.setAngle(Position.Retracted).unless(this::inConeMode));

    // vomit cube to first lvl
    m_driverController.x().onTrue(Sequences.vomit(m_intake, m_wheels));

    // pick a cone from the feeder station
    m_driverController
        .y()
        .toggleOnTrue(
            Sequences.PickConeFromFeeder(m_elevator, m_pivotArm, m_gripper)
                .unless(this::inCubeMode));

    m_driverController.y().toggleOnFalse(m_elevator.extendTo(Level.Down).unless(this::inCubeMode));

    // should enable vision mode
    // m_driverController.leftBumper().whileTrue(m_drive.setVisionMode(Mode.Last));
    // m_driverController.rightBumper().whileTrue(m_drive.setVisionMode(Mode.New));

    // m_driverController.rightBumper().onTrue(invertJoystick());

    // activate buddyClimb
    m_driverController.start().onTrue(m_buddyClimb.activate());

    m_driverController.povUp().onTrue(m_drive.balance());

    // launches cube to right lvl if in cube mode, and cone if in cone mode
    m_coDriverController.a().onTrue(m_wheels.launchTo().alongWith(m_rgbPanel.purpleCommand()));

    m_coDriverController
        .b()
        .onTrue(
            Commands.either(
                m_gripper.setGripperState(false),
                m_gripper.setGripperState(true),
                m_gripper::gripperState));

    m_coDriverController
        .x()
        .onTrue(
            Commands.either(
                Sequences.setTargetSecondIntake(m_intake, m_wheels)
                    .alongWith(m_rgbPanel.purpleCommand()),
                Sequences.scoreConeSecond(m_elevator, m_pivotArm, m_gripper)
                    .alongWith(m_rgbPanel.yellowCommand()),
                this::inCubeMode));

    m_coDriverController
        .y()
        .onTrue(
            Commands.either(
                Sequences.setTargetThirdIntake(m_intake, m_wheels)
                    .alongWith(m_rgbPanel.purpleCommand()),
                Sequences.scoreConeThird(m_elevator, m_pivotArm, m_gripper)
                    .alongWith(m_rgbPanel.yellowCommand()),
                this::inCubeMode));

    m_coDriverController.leftBumper().onTrue(m_elevator.extendTo(Elevator.Level.Down));
    m_coDriverController.rightBumper().onTrue(m_gripper.defaultWinch());

    // toggle robot modes
    m_coDriverController
        .start()
        .onTrue(
            Sequences.SwitchToCube(m_elevator, m_intake, m_pivotArm)
                .andThen(this.setMode(RobotMode.Cube))
                .andThen(m_rgbPanel.purpleCommand())
                .alongWith(new PrintCommand("Cube Mode"))
                .unless(this::inCubeMode));
    m_coDriverController
        .back()
        .onTrue(
            Sequences.SwitchToCone(m_elevator, m_intake, m_pivotArm, m_gripper)
                .andThen(this.setMode(RobotMode.Cone))
                .andThen(m_rgbPanel.yellowCommand())
                .alongWith(new PrintCommand("Cone Mode"))
                .unless(this::inConeMode));

    m_coDriverController.povLeft().onTrue(m_drive.moveScorePosition(false));
    m_coDriverController.povRight().onTrue(m_drive.moveScorePosition(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
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

  public static CommandXboxController getPilot() {
    return m_driverController;
  }

  public static CommandXboxController getCoPilot() {
    return m_coDriverController;
  }

  /**
   * starting here, commands define complex paths for autonomous aka with events
   *
   * @return the autonomous path
   */
  public Command runPathBalance() {
    eventMap.clear();
    eventMap.put("balance", new PrintCommand("balance")); // m_drive.balance()

    FollowPathWithEvents balance =
        new FollowPathWithEvents(
            m_drive.followPathCommand(pathBalance, true, true), pathBalance.getMarkers(), eventMap);

    return balance;
  }

  public Command stop() {
    Command stopComamnd = m_drive.driveWithSpeed(0, 0, 0);
    return stopComamnd;
  }

  public Command shootCube() {
    return m_intake
        .setAngle(Position.Launch)
        .andThen(new WaitCommand(3))
        .andThen(m_wheels.setTargetLevel(WheelLevel.Third))
        .andThen(m_wheels.launchTo());
  }
}
