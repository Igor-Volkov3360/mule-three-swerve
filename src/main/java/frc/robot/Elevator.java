// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final CANSparkMax m_elevator = new CANSparkMax(kElevatorId, MotorType.kBrushless);

  private final SparkMaxPIDController m_pidController = m_elevator.getPIDController();
  /** Creates a new Elevator. */
  public Elevator() {

    m_elevator.restoreFactoryDefaults();
  }

  public void up(double height) {
    m_pidController.setReference(height, CANSparkMax.ControlType.kSmartMotion);
  }

  public void down() {
    m_elevator.set(kDownCurrent);
  }

  public Command upCommand(double meters) {
    return this.run(() -> this.up(meters));
  }

  public Command downCommand() {
    return this.run(this::down);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
