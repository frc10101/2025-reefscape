// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import subsystems.ICEE;
import subsystems.nDexter;

public class RobotContainer {

  private nDexter dec = new nDexter();
  private ICEE icee = new ICEE();
  private CommandPS4Controller controller = new CommandPS4Controller(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    dec.setDefaultCommand(dec.runOnPID());
    Trigger x_bot = controller.cross();
    x_bot.onTrue(Commands.runOnce(() -> dec.setTarget(0, 0)));
    Trigger circle_bot = controller.circle();
    circle_bot.onTrue(Commands.runOnce(() -> dec.setTarget(50, 100)));
    Trigger square_bot = controller.square();
    square_bot.onTrue(Commands.runOnce(() -> dec.setTarget(100, .50)));
    Trigger triangle_bot = controller.triangle();
    triangle_bot.onTrue(Commands.runOnce(() -> dec.setTarget(100, 100)));

    Trigger iCeeTriggerIn = controller.L1();
    iCeeTriggerIn.whileTrue(icee.runOnPIDIn());
    Trigger iCeeTriggerOut = controller.L1();
    iCeeTriggerOut.whileTrue(icee.runOnPIDOut());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
