// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final CANSparkMax m_elevator = new CANSparkMax(kElevatorId, MotorType.kBrushless);
  /** Creates a new Elevator. */
  public Elevator() {

    m_elevator.restoreFactoryDefaults();
  }

  public void up() {
    m_elevator.set(kUpCurrent);
  }

  public void down() {
    m_elevator.set(kDownCurrent);
  }

  public Command upCommand() {
    return this.run(this::up);
  }

  public Command downCommand() {
    return this.run(this::down);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
