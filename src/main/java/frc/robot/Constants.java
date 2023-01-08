// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.math.geometry.Translation2d;

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

  public static class DriveTrain {
    public static final double kMaxModuleSpeed = 4.0;
    public static final double kMaxSpeedX = 4.0;
    public static final double kMaxSpeedY = 4.0;
    public static final double kMaxSpeetRot = 30.0;

    public static final double kDeadband = 0.05;
  }

  public static class HyperionSwerveModule {
    public static final int[] kDriveMotorIds = {7, 8, 10};
    public static final int[] kTurnMotorIds = {6, 5, 9};
    public static final int[] kAnalogZero = {412, 875, 738};
    public static final boolean[] kSensorPhases = {false, true, false};
    public static final InvertType[] kInvertType = {
      InvertType.None, InvertType.InvertMotorOutput, InvertType.InvertMotorOutput
    };

    public static final Translation2d[] kLocations = {
      new Translation2d(0.28, 0.0), new Translation2d(-0.28, 0.28), new Translation2d(-0.28, -0.28)
    };

    public static final double kAnalogToDeg = 360.0 / 1024;
    public static final double kDegToAnalog = 1.0 / kAnalogToDeg;

    public static final double kTickToMeter = 1.0 / 3243;
    public static final double kTickToMeterPerS = 10.0 * kTickToMeter;

    public static final double kMeterPerSToTick = 1.0 / kTickToMeterPerS;

    public static final double kTurnKp = 16.0;
    public static final double kTurnKi = 0.0;
    public static final double kTurnKd = 320.0;
    public static final double kTurnIZone = 0.0;

    public static final double kDriveKp = 4.0;
    public static final double kDriveKi = 0.0;
    public static final double kDriveKd = 0.0;
    public static final double kDriveKf = 0.5;
    public static final double kDriveIZone = 0.0;
  }
}
