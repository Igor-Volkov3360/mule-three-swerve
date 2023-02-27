// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is a subsystem that controls the RGB LED on the RoboRIO
// it works by creating a binary number from the three digital outputs,
// the arduino decodes this number and sets the LED accordingly
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem for the RGBPanel */
public class RGBControl extends SubsystemBase {

  private static DigitalOutput select1 = new DigitalOutput(0);
  private static DigitalOutput select2 = new DigitalOutput(1);
  private static DigitalOutput select3 = new DigitalOutput(2);
  private static PowerDistribution m_pdp = new PowerDistribution(20, ModuleType.kRev);
  private boolean m_state = false;

  /** Creates a new RGBControl. */
  public RGBControl() {
    m_pdp.setSwitchableChannel(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pdp.setSwitchableChannel(m_state);
  }

  public Command Command3360() {
    return this.run(
        () -> {
          select1.set(true);
          select2.set(false);
          select3.set(false);
        });
  }

  public Command blueCommand() {
    return this.run(
        () -> {
          select1.set(false);
          select2.set(true);
          select3.set(true);
        });
  }

  public Command greenCommand() {
    return this.run(
        () -> {
          select1.set(true);
          select2.set(false);
          select3.set(true);
        });
  }

  public Command redCommand() {
    return this.run(
        () -> {
          select1.set(false);
          select2.set(false);
          select3.set(true);
        });
  }

  public Command orangeCommand() {
    return this.run(
        () -> {
          select1.set(false);
          select2.set(true);
          select3.set(false);
        });
  }

  public Command purpleCommand() {
    return this.runOnce(
        () -> {
          select1.set(true);
          select2.set(true);
          select3.set(false);
        });
  }

  public Command teamCOmmand() {
    return this.runOnce(
        () -> {
          select1.set(false);
          select2.set(false);
          select3.set(false);
        });
  }

  public Command onPdp() {
    return this.runOnce(() -> m_state = !m_state);
  }
}
