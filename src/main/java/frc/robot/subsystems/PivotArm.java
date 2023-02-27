// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotArm extends SubsystemBase {

  // Subsystem parameters
  private static final int kPivotId = 16;
  private static final double kNativeToRad = 1.0;
  private static final double kNominalVolt = 10.0;

  public final double kUp = 1.8; // when the gripper is PARRALLEL to the ground
  public final double kDown = 0.0; // when the gripper is PERPENDICULAR to the ground
  private static final double kCube = 0.8;
  private double m_target = kDown;
  private static final double kMultiplier = 0.25;
  private static final double kCubeCounter = 0.05;

  // Member objects
  private final CANSparkMax m_pivot = new CANSparkMax(kPivotId, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_pivot.getEncoder();

  /** Creates a new PivotArm. */
  public PivotArm() {

    m_pivot.restoreFactoryDefaults();
    m_pivot.enableVoltageCompensation(kNominalVolt);
    m_encoder.setPositionConversionFactor(kNativeToRad);
    m_pivot.burnFlash();
  }

  @Override
  public void periodic() {
    m_pivot.set(motorSpeed() * kMultiplier);
    // System.out.println(motorSpeed());
    System.out.println(m_target);
  }

  /***
   * This function calculates the speed that the motor need to turn at
   * @return the deseried target - the current position
   */
  private double motorSpeed() {
    return (m_target - m_encoder.getPosition()) / 2;
  }

  /**
   * This function sets the desired angle for the pivot arm
   *
   * @param upDown true is up, false is down
   * @return Desired angle for the pivot arm
   */
  public Command setTarget(String position) {
    return this.runOnce(
        () -> {
          if (position == "up") m_target = kUp;
          else if (position == "down") m_target = kDown;
          else if (position == "cube") m_target = kCube;
        });
  }
  // mini neo = 110rpm at 100%
  // neo 567 at 100%
  public Command compensate(double position, double speed) {
    return setTarget("down");
  }

  public double getTarget() {
    return m_target;
  }
}
