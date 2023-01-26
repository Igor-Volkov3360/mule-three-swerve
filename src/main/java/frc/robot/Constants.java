// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.HyperionSwerveModule.HyperionSwerveModuleConfig;
import frc.robot.subsystems.WCPSwerveModule.WCPSwerveModuleConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Gripper {

    public static final int kGripMotorId = 0;

    public static final double kGripKp = 0;
    public static final double kGripKf = 0;

    public static final double kOpenCurrent = 0;
    public static final double kCloseCurrent = 0;
  }

  public static class DriveTrain {
    public static final double kMaxModuleSpeed = 2.0;

    public static final double kMaxSpeedX = 2.0;
    public static final double kMaxSpeedY = 2.0;
    public static final double kMaxSpeedRot = 270.0;

    public static final double kMaxAccTrans = 8.0;
    public static final double kMaxAccRot = 90.0;

    public static final double kJoystickDeadband = 0.15;

    public static final double kHoloKP = 2.0;
    public static final double kHoloKI = 0.0;
    public static final double kHoloKD = 0.0;

    public static final double kRotKP = 8.0;
    public static final double kRotKI = 0.0;
    public static final double kRotKD = 0.0;

    public static final int kPathServerPort = 5811;
  }

  public static class WCPSwerveModule {
    public static final WCPSwerveModuleConfig[] kConfigs = {};
    public static final Translation2d[] kLocations = {};
  }

  public static class HyperionSwerveModule {

    public static final HyperionSwerveModuleConfig[] kConfigs = {
      new HyperionSwerveModuleConfig(7, 6, 0, 455, InvertType.InvertMotorOutput, false),
      new HyperionSwerveModuleConfig(8, 5, 1, 871, InvertType.InvertMotorOutput, true),
      new HyperionSwerveModuleConfig(10, 9, 2, 721, InvertType.InvertMotorOutput, false)
    };

    public static final Translation2d[] kLocations = {
      new Translation2d(0.28, 0.0), new Translation2d(-0.28, 0.28), new Translation2d(-0.28, -0.28)
    };

    public static final double kAnalogToDeg = 360.0 / 1024;
    public static final double kDegToAnalog = 1.0 / kAnalogToDeg;

    public static final double kTickToMeter = 1.0 / 3243;
    public static final double kTickToMeterPerS = 10.0 * kTickToMeter;

    public static final double kMeterPerSToTick = 1.0 / kTickToMeterPerS;

    public static final double kTurnKp = 10.0;
    public static final double kTurnKi = 0.0;
    public static final double kTurnKd = 0.0;
    public static final double kTurnIZone = 0.0;

    public static final double kDriveKp = 3.0;
    public static final double kDriveKi = 0.0;
    public static final double kDriveKd = 0.0;
    public static final double kDriveKf = 0.0; // 0.5;
    public static final double kDriveIZone = 0.0;
  }
}
