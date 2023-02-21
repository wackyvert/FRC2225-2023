// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ChargeSwitch extends CommandBase{
  /** Creates a new ChargeSwitch. */
  public ChargeSwitch() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.chargeMode^=true;
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
