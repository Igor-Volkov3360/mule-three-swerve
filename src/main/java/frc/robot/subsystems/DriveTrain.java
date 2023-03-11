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
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionMeasurement;
import frc.robot.subsystems.WCPSwerveModule.WCPSwerveModuleFactory;
import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {

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
  public double YScoringPos = 0.5;

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

  private final Vision m_vision;

  LinearFilter m_xAccel = LinearFilter.movingAverage(10);

  // Process variables
  private double m_lastVisionTimestamp = -1.0;
  private PathPlannerTrajectory m_scoringTrajectory = null;

  /** Creates a new DriveTrain. */
  public DriveTrain(Vision vision) {

    m_vision = vision;

    // Reset gyro on code startup (Required as odometry starts at 0)
    m_gyro.calibrate();
    m_gyro.reset();
    // Run path planning server
    PathPlannerServer.startServer(kPathServerPort);
  }

  @Override
  public void periodic() {
    // Call module periodic

    inDeadband();

    for (final var module : m_modules) {
      module.periodic();
    }

    // Update odometry on each code loop
    m_odometry.update(m_gyro.getRotation2d(), this.getModulePositions());

    // Add vision measurement if it's available
    VisionMeasurement visionMes = m_vision.getMeasurement();
    if (visionMes != null && visionMes.m_timestamp != m_lastVisionTimestamp) {
      var visionPose =
          new Pose2d(
              visionMes.m_pose.getTranslation(), m_odometry.getEstimatedPosition().getRotation());

      if (m_lastVisionTimestamp < 0.0) {
        // Reset odometry to vision measurement on first observation
        m_odometry.resetPosition(m_gyro.getRotation2d(), this.getModulePositions(), visionPose);
      } else {
        m_odometry.addVisionMeasurement(visionPose, visionMes.m_timestamp);
      }
      m_lastVisionTimestamp = visionMes.m_timestamp;
    }

    // System.out.println(m_accelerometer.getX() + "         " + m_accelerometer.getZ());
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

  // More complex path with holonomic rotation. Non-zero starting velocity Max velocity of 4 m/s and
  // max accel of 3 m/s^2
  private PathPlannerTrajectory onTheFlyToScoringPos() {

    final var alliance = DriverStation.getAlliance();
    final var scoringDirDeg = alliance == Alliance.Blue ? 180.0 : 0.0;
    final var scoringX = alliance == Alliance.Blue ? 2.0 : 14.7;

    var scoringPos =
        new PathPoint(
            new Translation2d(scoringX, this.YScoringPos),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(scoringDirDeg));

    return PathPlanner.generatePath(new PathConstraints(4, 4), getOnTheFlyStart(), scoringPos);
  }

  private PathPoint getOnTheFlyStart() {
    var translation = m_odometry.getEstimatedPosition().getTranslation();
    var holonomicRot = m_odometry.getEstimatedPosition().getRotation();
    var chassisSpeed = m_kinematics.toChassisSpeeds(getModuleStates());

    var velocityVector =
        new Translation2d(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond);
    var fieldSpeed = velocityVector.rotateBy(holonomicRot);

    var vx = fieldSpeed.getX();
    var vy = fieldSpeed.getY();
    var magnitude = Math.sqrt(vx * vx + vy * vy);
    var direction = Rotation2d.fromRadians(Math.atan2(vy, vx));

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

  private boolean inDeadband() {
    return m_gyro.getAngle() < 0.2 && m_gyro.getAngle() > -0.2;
  }

  public Command balance() {
    return this.run(() -> drive(-0.4, 0, 0, true))
        .until(this::inAngle)
        .andThen(this.run(() -> drive(-0.2, 0, 0, true)))
        .until(this::parallel)
        .andThen(this.runOnce(() -> drive(0, 0, 0, true)));
  }

  public boolean inAngle() {
    return filteredX > 0.1;
  }

  public boolean parallel() {
    return filteredX < 0.1;
  }

  public Command goToTargetGoal() {
    return Commands.sequence(
        this.runOnce(() -> m_scoringTrajectory = this.onTheFlyToScoringPos()),
        this.followPathCommand(m_scoringTrajectory, false, false));
  }

  /**
   * @param toTheRight true means robot goes right
   */
  public Command moveScorePosition(boolean toTheRight) {
    return this.runOnce(() -> incrementScorePosition(toTheRight));
  }
  /**
   * @param direction true means robot goes right
   */
  public void incrementScorePosition(boolean direction) {
    if (direction) YScoringPos += 0.6;
    else YScoringPos -= 0.6;

    // clamps around min and max value to insure we don't run into the walls
    MathUtil.clamp(YScoringPos, minYScoringPos, maxYScoringPos);
  }
  // sets the Y target position to the robot's closest grid
  public void setToCLosestGoal() {
    double YCurrentPos = m_odometry.getEstimatedPosition().getY();
    double YCurrentBox = (YCurrentPos - minYScoringPos) / scoringGridIncrements;
    YScoringPos = (YCurrentBox * scoringGridIncrements) + minYScoringPos;
  }
}
