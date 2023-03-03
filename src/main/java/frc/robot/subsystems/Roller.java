// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {

  // Subsystem parameters
  private static final int kRollerId = 17;

  private static final double kRollerPercentCube = 0.7;
  private static final double kRollerPercentCone = 1;
  public static final double kCurrentThreshold = 35.0;

  // Member objects
  private final CANSparkMax m_roller = new CANSparkMax(kRollerId, MotorType.kBrushless);
  private final LinearFilter m_currentFilter = LinearFilter.singlePoleIIR(1.0, 1.0 / 50.0);

  /** Creates a new Roller. */
  public Roller() {

    m_roller.restoreFactoryDefaults();
    m_roller.setInverted(true);
    m_roller.burnFlash();

    // Stop intake by default
    this.setDefaultCommand(this.stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("   roller current " + m_roller.getOutputCurrent());
    if (DriverStation.isDisabled()) {
      m_currentFilter.reset();
    }
  }

  public Command stop() {
    return this.run(() -> m_roller.set(0.0));
  }

  public Command spin(String gamePiece) {
    return this.run(
        () -> {
          if (gamePiece == "cube") m_roller.set(kRollerPercentCube);
          else if (gamePiece == "cone") m_roller.set(kRollerPercentCone);
          else if (gamePiece == "reverse") m_roller.set(-kRollerPercentCone);
        });
  }

  public boolean isJammed() {
    return m_currentFilter.calculate(m_roller.getOutputCurrent()) > kCurrentThreshold;
  }
}
