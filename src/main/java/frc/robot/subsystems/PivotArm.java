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

  private static final double kUp = 2; // when the gripper is PARRALLEL to the ground
  private static final double kDown = 0.1; // when the gripper is PERPENDICULAR to the ground
  private double m_target = kDown;
  private static final double kMultiplier = 0.3;

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
    // This method will be called once per scheduler run

    // Set target to current when robot is disabled to prevent sudden motion on enable
    /*if (DriverStation.isDisabled()) {
      m_target = m_encoder.getPosition();
    }
    */
    System.out.println(motorSpeed() * kMultiplier);
    m_pivot.set(motorSpeed() * kMultiplier);
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
  public Command setTarget(boolean upDown) {
    return this.runOnce(
        () -> {
          if (upDown) m_target = kUp;
          else m_target = kDown;
        });
  }

  private double ajustEncoder() {
    return (1 - m_encoder.getPosition()) / 1.5;
  }
}
