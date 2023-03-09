// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BuddyClimb extends SubsystemBase {

  private static final int buddyLeftId = 10;
  private static final int buddyRightId = 9;

  private CANSparkMax buddyLeft = new CANSparkMax(buddyLeftId, MotorType.kBrushless);
  private CANSparkMax buddyRight = new CANSparkMax(buddyRightId, MotorType.kBrushless);

  /** Creates a new BuddyClimb. */
  public BuddyClimb() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
