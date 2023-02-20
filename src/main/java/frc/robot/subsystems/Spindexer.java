// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

  // Subsystem parameters
  private static final int kTableId = 11;
  private static final int kRollerId = 12;
  private static final int kBladeChannel = 0;
  private static final int kSwitchChannel = 9;

  private static double kTablePercent = 0.5;
  private static double kRollerPercent = 0.5;
  private static double kUpDeg = 90.0;
  private static double kDownDeg = 0.0;

  private static boolean kIndexedBool = true;

  // Member objects
  private final CANSparkMax m_table = new CANSparkMax(kTableId, MotorType.kBrushless);
  private final CANSparkMax m_roller = new CANSparkMax(kRollerId, MotorType.kBrushless);
  private final Servo m_blade = new Servo(kBladeChannel);
  private final DigitalInput m_switch = new DigitalInput(kSwitchChannel);

  // Process variables
  private double m_tablePercent = 0.0;
  private double m_rollerPercent = 0.0;
  private double m_bladeDeg = kUpDeg;

  /** Creates a new Spindexer. */
  public Spindexer() {

    m_roller.restoreFactoryDefaults();
    m_table.restoreFactoryDefaults();

    // Stop everything by default
    this.setDefaultCommand(this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_roller.set(m_rollerPercent);
    m_table.set(m_tablePercent);
    m_blade.setAngle(m_bladeDeg);
  }

  /**
   * Check if the game piece is indexed against the blade
   *
   * @return game piece is indexed
   */
  public boolean atIndex() {
    return m_switch.get() == kIndexedBool;
  }

  /**
   * Spin the roller and table with the blade lifted
   *
   * @return instant command
   */
  public Command spin() {
    return this.run(
        () -> {
          m_blade.setAngle(kUpDeg);
          m_roller.set(kRollerPercent);
          m_table.set(kTablePercent);
        });
  }

  /**
   * Spin the table with the blade lowered until the game piece is indexed
   *
   * @return blocking command
   */
  public Command index() {
    return this.run(
            () -> {
              m_blade.setAngle(kDownDeg);
              m_roller.set(0.0);
              m_table.set(kTablePercent);
            })
        .until(this::atIndex);
  }

  /**
   * Stop all motion
   *
   * @return instant command
   */
  public Command stop() {
    return this.run(
        () -> {
          m_roller.set(0.0);
          m_table.set(0.0);
        });
  }
}
