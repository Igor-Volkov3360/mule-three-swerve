// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

  // Subsystem parameters
  private static final int kTableId = 12;
  private static final int kRollerId = 11;
  private static final int kBladeChannel = 0;

  private static double kTablePercent = -0.5;
  private static double kRollerPercent = 0.5;
  private static double kOut = 0.6; // out is outside of the tub
  private static double kIn = 0.25;

  // Member objects
  private final CANSparkMax m_table = new CANSparkMax(kTableId, MotorType.kBrushless);
  private final CANSparkMax m_roller = new CANSparkMax(kRollerId, MotorType.kBrushless);
  private final Servo m_blade = new Servo(kBladeChannel);

  /** Creates a new Spindexer. */
  public Spindexer() {

    m_roller.restoreFactoryDefaults();
    m_table.restoreFactoryDefaults();

    m_roller.setInverted(true);

    // Stop everything by default
    this.setDefaultCommand(this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Spin the roller and table with the blade lifted
   *
   * @return instant command
   */
  public Command spin() {
    return this.run(
        () -> {
          m_blade.set(kOut);
          m_table.set(kTablePercent);
          m_roller.set(kRollerPercent);
        });
  }

  // ne pas tourner si cube

  /**
   * Stop all motion
   *
   * @return instant command
   */
  public Command stop() {
    return this.run(
        () -> {
          m_blade.set(kOut);
          m_roller.set(0.0);
          m_table.set(0.0);
        });
  }

  public Command index() {
    return this.run(
        () -> {
          m_blade.set(kIn);
          m_table.set(kTablePercent);
        });
  }

  public Command baldeOut() {
    return this.run(() -> m_blade.set(kOut));
  }
}
