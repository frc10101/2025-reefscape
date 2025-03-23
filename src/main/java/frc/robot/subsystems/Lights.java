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

  public Lights() {
    configureCANdle();
  }

  private void configureCANdle() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = true;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.1;
    config.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(config, 100);
  }

  private void setColors(int red, int green, int blue, double strength) {
    red = clamp(red, 0, 255);
    green = clamp(green, 0, 255);
    blue = clamp(blue, 0, 255);
    strength = clamp(strength, 0, 1);

    candle.setLEDs(red, green, blue);
    candle.modulateVBatOutput(strength);
  }

  private int clamp(int value, int min, int max) {
    return Math.max(min, Math.min(max, value));
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  public Command setColorsCommand(int red, int green, int blue, double strength) {
    return runOnce(() -> setColors(red, green, blue, strength));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
