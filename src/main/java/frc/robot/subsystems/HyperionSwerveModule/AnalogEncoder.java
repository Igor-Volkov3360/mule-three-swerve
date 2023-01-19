// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HyperionSwerveModule;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Emulates the analog encoder of a TalonSRX using a RIO input */
public class AnalogEncoder {

  // Maximum analog input value
  private static final double ANALOG_RANGE = 5.0;

  // Maximum digital output value
  private static final double DIGITAL_RANGE = 1023.0;

  private final AnalogInput m_analogInput;

  // Input variation considered as a wrap (V)
  private final double m_wrapThreshold;

  private final GenericEntry m_valueEntry;

  private double m_lastVoltage = -1.0;
  private double m_rollover = 0.0;

  /**
   * Constructs an AnalogEncoder
   *
   * @param channel analog input channel on the RIO
   * @param wrapThreshold theshold considered as a wrap (V)
   */
  public AnalogEncoder(int channel, double wrapThreshold, String entryName) {
    m_analogInput = new AnalogInput(channel);
    m_wrapThreshold = wrapThreshold;
    m_valueEntry = Shuffleboard.getTab("AnalogEncoder").add(entryName, -1.0).getEntry();
  }

  /**
   * Gets the current digitalized angle
   *
   * @return Wrapped absolute angle (1 rot = 1023)
   */
  public double getAngle() {
    final double currentVoltage = m_analogInput.getVoltage();

    if (m_lastVoltage > -0.5) {
      // Skips wrap check if first iteration

      final double delta = m_lastVoltage - currentVoltage;
      if (Math.abs(delta) > m_wrapThreshold) {
        m_rollover += ANALOG_RANGE * Math.signum(delta);
      }
    }
    m_lastVoltage = currentVoltage;

    final double wrappedVoltage = currentVoltage + m_rollover;
    final double digitalValue = DIGITAL_RANGE * wrappedVoltage / ANALOG_RANGE;

    m_valueEntry.setDouble(digitalValue);
    return digitalValue;
  }
}
