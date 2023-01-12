// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Gripper.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  private final TalonSRX m_gripMotor = new TalonSRX(kGripMotorId);
  /** Creates a new Gripper. */
  public Gripper() {

    m_gripMotor.configFactoryDefault();

    m_gripMotor.config_kP(0, kGripKp);
    m_gripMotor.config_kF(0, kGripKf);
  }

  public void open() {

    m_gripMotor.set(ControlMode.Current, kOpenCurrent);
  }

  public void close() {

    m_gripMotor.set(ControlMode.Current, kCloseCurrent);
  }

  public Command openCommand() {

    return this.run(this::open);
  }

  public Command closeCommand() {

    return this.run(this::close);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
