// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrain.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.HyperionSwerveModule.HyperionSwerveModuleFactory;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionMeasurement;
import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {

  private final SwerveModuleFactory m_moduleFactory = new HyperionSwerveModuleFactory();
  private final SwerveModule[] m_modules = m_moduleFactory.createModules();
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_moduleFactory.getLocations());

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private final SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          m_kinematics, m_gyro.getRotation2d(), this.getModulePositions(), new Pose2d());

  private final HolonomicDriveController m_holonomicController =
      new HolonomicDriveController(
          new PIDController(kHoloKP, kHoloKI, kHoloKD),
          new PIDController(kHoloKP, kHoloKI, kHoloKD),
          new ProfiledPIDController(
              kRotKP, kRotKI, kRotKD, new Constraints(kMaxSpeedRot, kMaxAccRot)));

  private final Vision m_vision;
  private double m_lastVisionTimestamp = -1.0;

  /** Creates a new DriveTrain. */
  public DriveTrain(Vision vision) {

    m_vision = vision;

    // Reset gyro on code startup (Required as odometry starts at 0)
    m_gyro.reset();

    // Run path planning server
    PathPlannerServer.startServer(kPathServerPort);
  }

  @Override
  public void periodic() {
    // Update odometry on each code loop
    m_odometry.update(m_gyro.getRotation2d(), this.getModulePositions());

    // Add vision measurement if it's available
    VisionMeasurement visionMes = m_vision.getMeasurement();
    if (visionMes != null && visionMes.m_timestamp != m_lastVisionTimestamp) {
      m_odometry.addVisionMeasurement(visionMes.m_pose, visionMes.m_timestamp);
      m_lastVisionTimestamp = visionMes.m_timestamp;
    }
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
                    xSpeed, ySpeed, rotSpeed, m_gyro.getRotation2d())
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
   * @return Scaled joystick value (physical units)
   */
  private static double scaleJoystickInput(DoubleSupplier valueSupplier, double maxValue) {
    double value = valueSupplier.getAsDouble();

    return Math.abs(value) > kJoystickDeadband
        ? value * value * Math.signum(value) * maxValue
        : 0.0;
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

    return this.run(
        () ->
            this.drive(
                DriveTrain.scaleJoystickInput(xSpeed, kMaxSpeedX),
                DriveTrain.scaleJoystickInput(ySpeed, kMaxSpeedY),
                DriveTrain.scaleJoystickInput(rotSpeed, kMaxSpeedRot),
                fieldRelative));
  }

  /**
   * Command that follows a trajectory
   *
   * @param trajectory trajectory to follow
   * @return schedulable command
   */
  public Command followCommand(Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory, this::getPose, m_kinematics, m_holonomicController, this::drive, this);
  }

  public Command followPathCommand(PathPlannerTrajectory trajectory) {
    return new PPSwerveControllerCommand(
        trajectory,
        this::getPose,
        this.m_kinematics,
        new PIDController(kHoloKP, kHoloKI, kHoloKD),
        new PIDController(kHoloKP, kHoloKI, kHoloKD),
        new PIDController(kRotKP, kRotKI, kRotKD),
        this::drive,
        false,
        this);
  }
}
