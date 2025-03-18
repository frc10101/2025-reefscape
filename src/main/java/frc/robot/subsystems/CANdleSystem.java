// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleSystem extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.CANids.CANdleID);
  private final int LedCount = 300;

  private Animation m_toAnimate = null;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  public CANdleSystem() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  /* Wrappers so we can access the CANdle from the subsystem */
  public double getVbat() {
    return m_candle.getBusVoltage();
  }

  public double get5V() {
    return m_candle.get5VRailVoltage();
  }

  public double getCurrent() {
    return m_candle.getCurrent();
  }

  public double getTemperature() {
    return m_candle.getTemperature();
  }

  public void configBrightness(double percent) {
    m_candle.configBrightnessScalar(percent, 0);
  }

  public void configLos(boolean disableWhenLos) {
    m_candle.configLOSBehavior(disableWhenLos, 0);
  }

  public void configLedType(LEDStripType type) {
    m_candle.configLEDType(type, 0);
  }

  public void configStatusLedBehavior(boolean offWhenActive) {
    m_candle.configStatusLedState(offWhenActive, 0);
  }

  public Command haveCoral() {
    return runOnce(
        () -> {
          this.m_candle.setLEDs(202, 251, 19);
        });
  }

  public Command noCoral() {
    return runOnce(
        () -> {
          this.m_candle.setLEDs(193, 0, 193);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //     if(m_toAnimate == null) {
    //         m_candle.setLEDs((int)(joystick.getLeftTriggerAxis() * 255),
    //                           (int)(joystick.getRightTriggerAxis() * 255),
    //                           (int)(joystick.getLeftX() * 255));
    //     } else {
    //         m_candle.animate(m_toAnimate);
    //     }
    //     m_candle.modulateVBatOutput(joystick.getRightY());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
