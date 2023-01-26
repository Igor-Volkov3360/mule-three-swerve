// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final CANSparkMax m_leader = new CANSparkMax(kLeaderId, MotorType.kBrushless);
  private final CANSparkMax m_follower = new CANSparkMax(kFollowerId, MotorType.kBrushless);

  /** Creates a new Elevator. */
  public Elevator() {

    m_leader.restoreFactoryDefaults();
    m_leader.setInverted(kLeaderInverted);
    m_leader.burnFlash();

    m_follower.restoreFactoryDefaults();
    m_follower.follow(m_leader, kFollowerOpposeLeader);
    m_follower.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHeight(double meters) {
    // TODO: Implement smart motion position control
  }
}
