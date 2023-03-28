// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionMeasurement;
import frc.robot.subsystems.WCPSwerveModule.WCPSwerveModuleFactory;
import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {

  public enum Mode {
    New,
    Last,
    Disabled
  }

  // Subsystem parameters
  public static final double kMaxModuleSpeed = 4.0;

  public static final double kMaxSpeedX = 4.0;
  public static final double kMaxSpeedY = 4.0;
  public static final double kMaxSpeedRot = 360.0;

  public static final double kMaxAccTrans = 2.0 * kMaxModuleSpeed;
  public static final double kMaxAccRot = 4.0 * kMaxSpeedRot;

  public static final double kJoystickDeadband = 0.15;

  public static final double kHoloKP = 2.0;
  public static final double kHoloKI = 0.0;
  public static final double kHoloKD = 0.0;

  public static final double kRotKP = 8.0;
  public static final double kRotKI = 0.0;
  public static final double kRotKD = 0.0;

  public double filteredX = 0;
  public static final double XScoringPos = 2;
  public static final double minYScoringPos = 0.5;
  public static final double maxYScoringPos = 5;
  public static final double scoringGridIncrements = (maxYScoringPos - minYScoringPos) / 8;
  public double YScoringPos = 2.75;
  public boolean m_hadRecentVision = false;

  public Mode m_visionMode = Mode.Disabled;

  public static final int kPathServerPort = 5811;

  // Member objects
  private final SwerveModuleFactory m_moduleFactory = new WCPSwerveModuleFactory();
  private final SwerveModule[] m_modules = m_moduleFactory.createModules();
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_moduleFactory.getLocations());

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
  private final SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          m_kinematics, m_gyro.getRotation2d(), this.getModulePositions(), new Pose2d());

  Accelerometer m_accelerometer = new BuiltInAccelerometer();

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(kMaxAccTrans);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(kMaxAccTrans);
  private final SlewRateLimiter m_zLimiter = new SlewRateLimiter(kMaxAccRot);

  private Field2d m_field = new Field2d();

  private final Vision m_vision;

  LinearFilter m_xAccel = LinearFilter.movingAverage(30);

  // Process variables
  private double m_lastVisionTimestamp = -1.0;
  private Command m_scoringCommand = Commands.none();

  /** Creates a new DriveTrain. */
  public DriveTrain(Vision vision) {

    m_vision = vision;

    // Reset gyro on code startup (Required as odometry starts at 0)
    m_gyro.calibrate();
    m_gyro.reset();
    // Run path planning server
    PathPlannerServer.startServer(kPathServerPort);
    SmartDashboard.putData("field", m_field);
  }

  @Override
  public void periodic() {
    // System.out.println("isBalanced : " + isBalanced());
    // resetOdometry();
    // Call module periodic
    filteredX = m_xAccel.calculate(m_accelerometer.getX());

    for (final var module : m_modules) {
      module.periodic();
    }

    // Update odometry on each code loop
    m_odometry.update(m_gyro.getRotation2d(), this.getModulePositions());

    // Add vision measurement if it's available
    VisionMeasurement visionMes = m_vision.getMeasurement();
    if (visionMes != null && visionMes.m_timestamp != m_lastVisionTimestamp) {
      var visionPose =
          new Pose2d(visionMes.m_pose.getTranslation(), visionMes.m_pose.getRotation());

      if (m_lastVisionTimestamp < 0.0) {
        // Reset odometry to vision measurement on first observation
        m_odometry.resetPosition(m_gyro.getRotation2d(), this.getModulePositions(), visionPose);
      } else {
        m_odometry.addVisionMeasurement(visionPose, visionMes.m_timestamp);
      }
      m_lastVisionTimestamp = visionMes.m_timestamp;
    }
    m_hadRecentVision = (Timer.getFPGATimestamp() - m_lastVisionTimestamp < 0.5);

    SmartDashboard.putBoolean("vision reading", m_hadRecentVision);

    m_field.setRobotPose(m_odometry.getEstimatedPosition());
    // System.out.println(m_accelerometer.getZ());
  }
  /**
   * Gets the current position of all modules
   *
   * @return array of module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    var positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < positions.length; ++i) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    var states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < states.length; ++i) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  /**
   * Drives the robot in closed-loop velocity
   *
   * @param xSpeed Velocity along x (m/s)
   * @param ySpeed Velocity along y (m/s)
   * @param rotSpeed Rotation speed around x (deg/s)
   * @param fieldRelative Velocity are field relative
   */
  private void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    rotSpeed = Math.toRadians(rotSpeed);
    var moduleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, m_odometry.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxModuleSpeed);
    for (int i = 0; i < m_modules.length; ++i) {
      m_modules[i].setDesiredState(moduleStates[i]);
    }
  }

  /**
   * Drives the robot in closed-loop velocity
   *
   * @param moduleStates Swerve modules states
   */
  public void drive(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < m_modules.length; ++i) {
      m_modules[i].setDesiredState(moduleStates[i]);
    }
  }

  /**
   * Gets the current pose of the robot from the state estimator
   *
   * @return Field relative robot pose
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Scales a joystick input to make it suitable as a velocity value
   *
   * @param valueSupplier Raw joystick value supplier (-1, 1)
   * @param maxValue Maximum value in physical units
   * @param rateLimiter Slew rate limiter to limit rate of change
   * @return Scaled joystick value (physical units)
   */
  private static double scaleJoystickInput(
      DoubleSupplier valueSupplier, double maxValue, SlewRateLimiter rateLimiter) {

    double rawValue = valueSupplier.getAsDouble();
    double value =
        Math.abs(rawValue) > kJoystickDeadband ? rawValue * rawValue * Math.signum(rawValue) : 0.0;

    return rateLimiter.calculate(value * maxValue);
  }

  /**
   * Command that drives using axis from the provided suppliers
   *
   * @param xSpeed Velocity along x (-1, 1) supplier
   * @param ySpeed Velocity along y (-1, 1) supplier
   * @param rotSpeed Rotation speed around x (-1, 1) supplier
   * @param fieldRelative Velocity are field relative supplier
   * @return run command that runs forever
   */
  public Command driveCommand(
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rotSpeed,
      boolean fieldRelative) {

    return this.runOnce(
            () -> {
              this.m_xLimiter.reset(0.0);
              this.m_yLimiter.reset(0.0);
              this.m_zLimiter.reset(0.0);
            })
        .andThen(
            this.run(
                () ->
                    this.drive(
                        DriveTrain.scaleJoystickInput(xSpeed, kMaxSpeedX, m_xLimiter),
                        DriveTrain.scaleJoystickInput(ySpeed, kMaxSpeedY, m_yLimiter),
                        DriveTrain.scaleJoystickInput(rotSpeed, kMaxSpeedRot, m_zLimiter),
                        fieldRelative)));
  }

  /**
   * Command that drives along a trajectory
   *
   * @param trajectory path planner generated trajectory
   * @return blocking command
   */
  public Command followPathCommand(
      PathPlannerTrajectory trajectory, boolean resetOdometry, boolean useAllianceColour) {

    return this.runOnce(() -> this.resetOdometryToTrajectoryStart(trajectory))
        .unless(() -> !resetOdometry)
        .andThen(
            new PPSwerveControllerCommand(
                trajectory,
                this::getPose,
                this.m_kinematics,
                new PIDController(kHoloKP, kHoloKI, kHoloKD),
                new PIDController(kHoloKP, kHoloKI, kHoloKD),
                new PIDController(kRotKP, kRotKI, kRotKD),
                this::drive,
                useAllianceColour,
                this));
  }

  public PathPlannerTrajectory autoPathBalance() {

    final var alliance = DriverStation.getAlliance();
    final var moveDirDeg = alliance == Alliance.Blue ? 180.0 : 0.0;
    final var balanceX = alliance == Alliance.Blue ? 5.5 : 11.0;
    final var startX = alliance == Alliance.Blue ? 2.1 : 14.4;
    final var headingMove = alliance == Alliance.Blue ? 0.0 : 180.0;
    final var waypointY = 4.75;
    final var balanceY = 2.75;

    PathPoint startPoint =
        new PathPoint(
            new Translation2d(startX, 5),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(moveDirDeg));

    PathPoint move1 =
        new PathPoint(
            new Translation2d(balanceX, waypointY),
            Rotation2d.fromDegrees(headingMove),
            Rotation2d.fromDegrees(moveDirDeg));

    PathPoint balance =
        new PathPoint(
            new Translation2d(balanceX, balanceY),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(moveDirDeg));

    return PathPlanner.generatePath(new PathConstraints(2, 2), startPoint, move1, balance);
  }

  public PathPlannerTrajectory autoPathIntake(boolean firstMove) {

    final var alliance = DriverStation.getAlliance();
    final var intakeDirDeg = alliance == Alliance.Red ? 180.0 : 0.0;
    final var moveDirDeg = alliance == Alliance.Blue ? 180.0 : 0.0;
    final var intakeX = alliance == Alliance.Blue ? 6.8 : 9.7;
    final var startX = alliance == Alliance.Blue ? 2.1 : 14.4;
    final var waypoint1X = alliance == Alliance.Blue ? 4.5 : 12.0;
    final var waypoint2X = alliance == Alliance.Blue ? 5.5 : 11.0;
    final var headingMove = alliance == Alliance.Blue ? 0.0 : 180.0;
    final var waypointY = 4.75;
    final var intakeY = 4.4;

    PathPoint startPoint =
        new PathPoint(
            new Translation2d(startX, 5),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(moveDirDeg));

    PathPoint move1 =
        new PathPoint(
            new Translation2d(waypoint1X, waypointY),
            Rotation2d.fromDegrees(headingMove),
            Rotation2d.fromDegrees(moveDirDeg));

    PathPoint move2 =
        new PathPoint(
            new Translation2d(waypoint2X, intakeY),
            Rotation2d.fromDegrees(headingMove),
            Rotation2d.fromDegrees(intakeDirDeg));

    PathPoint intake =
        new PathPoint(
            new Translation2d(intakeX, intakeY),
            Rotation2d.fromDegrees(headingMove),
            Rotation2d.fromDegrees(intakeDirDeg));

    if (firstMove) {
      return PathPlanner.generatePath(new PathConstraints(0.8, 0.8), startPoint, move1, move2);
    } else {
      return PathPlanner.generatePath(new PathConstraints(1.5, 1.5), move2, intake);
    }
  }

  private PathPlannerTrajectory onTheFlyToIntake() {

    final var alliance = DriverStation.getAlliance();
    final var scoringDirDeg = alliance == Alliance.Blue ? 0.0 : 180.0;
    final var intakeX = alliance == Alliance.Blue ? 15.3 : 1.2;
    final var intakeY = alliance == Alliance.Blue ? 7.4 : 6.15;
    final var robotPos = m_odometry.getEstimatedPosition().getTranslation();
    double waypointX = alliance == Alliance.Blue ? 14.8 : 1.7;

    var waypoint1 =
        new PathPoint(
            new Translation2d(waypointX, intakeY),
            Rotation2d.fromDegrees(0.0),
            Rotation2d.fromDegrees(scoringDirDeg));

    var intake =
        new PathPoint(
            new Translation2d(intakeX, intakeY),
            Rotation2d.fromDegrees(0.0),
            Rotation2d.fromDegrees(scoringDirDeg));

    if (alliance == alliance.Blue && robotPos.getX() > 13) {
      return PathPlanner.generatePath(
          new PathConstraints(2, 2), getOnTheFlyStart(false), waypoint1, intake);
    } else if (alliance == alliance.Red && robotPos.getX() < 3.5) {
      return PathPlanner.generatePath(
          new PathConstraints(2, 2), getOnTheFlyStart(false), waypoint1, intake);
    } else return null;
  }

  /**
   * More complex path with holonomic rotation. Non-zero starting velocity Max velocity of 4 m/s and
   * max accel of 3 m/s^2
   *
   * @return a path to follow
   */
  private PathPlannerTrajectory onTheFlyToScoringPos() {

    final var alliance = DriverStation.getAlliance();
    final var scoringDirDeg = alliance == Alliance.Blue ? 183.0 : 3.0;
    final var scoringX = alliance == Alliance.Blue ? 2.1 : 14.4;
    final var robotPos = m_odometry.getEstimatedPosition().getTranslation();
    double waypointY = 4.75;
    double waypoint2X = alliance == Alliance.Blue ? 2.3 : 14.1;
    double waypointX = 5.5;
    double scorePosY = this.YScoringPos;
    var headingScore = Rotation2d.fromDegrees(90);
    var headingWay = Rotation2d.fromDegrees(180);

    // sets the right X coordinate according to color
    if (alliance == Alliance.Red) {
      waypointX = 13.0;
      headingWay = Rotation2d.fromDegrees(0);
    }
    // decides if the robots goes around the left or right side
    if (robotPos.getY() < 2.75) waypointY = 0.75;

    var waypoint1 =
        new PathPoint(
            new Translation2d(waypointX, waypointY),
            headingWay,
            Rotation2d.fromDegrees(scoringDirDeg));

    var waypoint2 =
        new PathPoint(
            new Translation2d(waypoint2X, waypointY),
            headingWay,
            Rotation2d.fromDegrees(scoringDirDeg));

    if (alliance == Alliance.Blue) {
      if (waypointY > scorePosY) headingScore = Rotation2d.fromDegrees(270);
    }
    if (alliance == Alliance.Red) {
      if (waypointY < scorePosY) headingScore = Rotation2d.fromDegrees(270);
    }
    var scoringPos =
        new PathPoint(
            new Translation2d(scoringX, this.YScoringPos),
            headingScore,
            Rotation2d.fromDegrees(scoringDirDeg));

    if (robotPos.getX() > 4.5 && robotPos.getX() < 12) {
      return PathPlanner.generatePath(
          new PathConstraints(3, 3), getOnTheFlyStart(false), waypoint1, waypoint2, scoringPos);
    } else if (robotPos.getX() > 2.5 && robotPos.getX() < 14) {
      return PathPlanner.generatePath(
          new PathConstraints(3, 3), getOnTheFlyStart(false), waypoint2, scoringPos);
    } else {
      return PathPlanner.generatePath(
          new PathConstraints(3, 3), getOnTheFlyStart(true), scoringPos);
    }
  }

  private PathPoint getOnTheFlyStart(boolean headingOverride) {
    var translation = m_odometry.getEstimatedPosition().getTranslation();
    var holonomicRot = m_odometry.getEstimatedPosition().getRotation();
    var chassisSpeed = m_kinematics.toChassisSpeeds(getModuleStates());

    var velocityVector =
        new Translation2d(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond);
    var fieldSpeed = velocityVector.rotateBy(holonomicRot);

    var vx = fieldSpeed.getX();
    var vy = fieldSpeed.getY();
    var magnitude = Math.sqrt(vx * vx + vy * vy);
    var direction =
        headingOverride ? Rotation2d.fromDegrees(90) : Rotation2d.fromRadians(Math.atan2(vy, vx));

    return new PathPoint(translation, direction, holonomicRot, magnitude);
  }

  /**
   * Reset the odometry current position to the trajectory initial state
   *
   * @param trajectory trajectory to reset odometry to
   */
  public void resetOdometryToTrajectoryStart(PathPlannerTrajectory trajectory) {
    final var start =
        PathPlannerTrajectory.transformStateForAlliance(
            trajectory.getInitialState(), DriverStation.getAlliance());
    m_odometry.resetPosition(m_gyro.getRotation2d(), this.getModulePositions(), start.poseMeters);
  }

  public Command balance() {
    return this.run(() -> drive(0.6, 0, 0, true))
        .until(this::inAngle)
        .andThen(this.run(() -> drive(0.3, 0, 0, true)))
        .until(this::parallel)
        .andThen(this.runOnce(() -> drive(0, 0, 0, true)));
  }

  public boolean inAngle() {
    return filteredX > 0.3;
  }

  public boolean parallel() {
    return filteredX < 0.3 && filteredX > -0.3;
  }

  /**
   * this command makes the robot go to the YScoringPos using onTheFlyToScoringPos
   *
   * @return commands that make the robot go to the right scoring pos
   */
  public Command goToTargetGoal() {
    return this.runOnce(
        () -> {
          final var traj = this.onTheFlyToScoringPos();
          m_scoringCommand = this.followPathCommand(traj, false, false);
          m_scoringCommand.schedule();
        });
  }

  public Command goToHpIntake() {
    return this.runOnce(
        () -> {
          final var traj = this.onTheFlyToIntake();
          if (traj != null) {
            m_scoringCommand = this.followPathCommand(traj, false, false);
            m_scoringCommand.schedule();
          }
        });
  }

  public Command runAuto1stMove() {
    return this.followPathCommand(autoPathIntake(true), false, false);
  }

  public Command runAuto2ndMove() {
    return this.followPathCommand(autoPathIntake(false), false, false);
  }

  public Command runAutoBalancePath() {
    return this.followPathCommand(autoPathBalance(), false, false);
  }

  public Command goToTargetCube() {
    return this.run(
        () -> {
          double xSpeed = m_vision.getCubeYpos();
          double ySpeed = m_vision.getCubeXpos();
          this.driveWithSpeed(xSpeed, ySpeed, 0);
        });
  }

  public Command stop() {
    return this.runOnce(() -> this.driveWithSpeed(0.0, 0.0, 0.0));
  }

  /**
   * @param toTheRight true means robot goes right, flips automatically with the color
   */
  public Command moveScorePosition(boolean toTheRight) {
    return this.runOnce(
        () -> {
          boolean temp = DriverStation.getAlliance() == Alliance.Red ? !toTheRight : toTheRight;
          this.incrementScorePosition(temp);
        });
  }

  /**
   * @param direction true means robot goes right
   */
  public void incrementScorePosition(boolean direction) {
    if (direction) YScoringPos += 0.5625;
    else YScoringPos -= 0.5625;
    System.out.println("incrementing" + YScoringPos);
    // clamps around min and max value to insure we don't run into the walls
    YScoringPos = MathUtil.clamp(YScoringPos, minYScoringPos, maxYScoringPos);
    double gridPos = ((YScoringPos - minYScoringPos) / 0.5625) + 1;
    if (DriverStation.getAlliance() == Alliance.Blue) gridPos = 10 - gridPos;
    SmartDashboard.putNumber("Grid Position (left to right", gridPos);
  }

  /* reset the yScoringPos to closest scoring area */
  public Command resetToCLosestScoringPos() {
    return this.runOnce(
        () -> {
          double YCurrentPos = m_odometry.getEstimatedPosition().getY();
          double YCurrentBox = (YCurrentPos - minYScoringPos) / scoringGridIncrements;
          YScoringPos = (Math.round(YCurrentBox) * scoringGridIncrements) + minYScoringPos;
        });
  }

  public Command autoBalance() {
    return this.run(() -> this.drive(1.0, 0.0, 0.0, true)).until(m_vision::RobotOnTargetBalance);
  }

  public Command driveWithSpeed(double speedX, double speedY, double rotation) {
    return this.run(() -> this.drive(speedX, speedY, rotation, true));
  }

  public Command driveWithSpeedWithTimeout(
      double speedX, double speedY, double rotation, double time) {
    return this.run(() -> this.drive(speedX, speedY, rotation, true)).withTimeout(time);
  }

  public boolean hasRecentTarget() {
    return Timer.getFPGATimestamp() - m_lastVisionTimestamp < 0.25;
  }

  public double timeSinceLastTarget() {
    return Timer.getFPGATimestamp() - m_lastVisionTimestamp;
  }
}
