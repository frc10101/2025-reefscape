// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private final CANdle candle = new CANdle(1, "rio");

  /** Creates a new Lights. */
  public Lights() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = true;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.1;
    config.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(config, 100);
  }

  private void setColors(int red, int green, int blue, double strength) {
    if (red < 0) red = 0;
    if (red > 255) red = 255;
    if (green < 0) green = 0;
    if (green > 255) green = 255;
    if (blue < 0) blue = 0;
    if (blue > 255) blue = 255;
    if (strength > 1) strength = 1;
    if (strength < 0) strength = 0;
    candle.setLEDs(red, green, blue);
    candle.modulateVBatOutput(strength);
  }

  public Command setColorsCommand(int red, int green, int blue, double strength) {
    return runOnce(() -> setColors(red, green, blue, strength));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
