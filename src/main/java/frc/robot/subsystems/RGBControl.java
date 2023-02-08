// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is a subsystem that controls the RGB LED on the RoboRIO
// it works by creating a binary number from the three digital outputs,
// the arduino decodes this number and sets the LED accordingly
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem for the RGBPanel */
public class RGBControl extends SubsystemBase {

  private static DigitalOutput select1 = new DigitalOutput(0);
  private static DigitalOutput select2 = new DigitalOutput(1);
  private static DigitalOutput select3 = new DigitalOutput(2);

  /** Creates a new RGBControl. */
  public RGBControl() {
    set3360();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void setTeam() {
    select1.set(false);
    select2.set(false);
    select3.set(false);
  }

  public static void set3360() {
    select1.set(true);
    select2.set(false);
    select3.set(false);
  }

  public static void setOrange() {
    select1.set(false);
    select2.set(true);
    select3.set(false);
  }

  public static void setPurple() {
    select1.set(true);
    select2.set(true);
    select3.set(false);
  }

  public static void setRed() {
    select1.set(false);
    select2.set(false);
    select3.set(true);
  }

  public static void setGreen() {
    select1.set(true);
    select2.set(false);
    select3.set(true);
  }

  public static void setBlue() {
    select1.set(false);
    select2.set(true);
    select3.set(true);
  }
}
