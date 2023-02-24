// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CloseClaw extends CommandBase {
  /** Creates a new CloseClaw. */
  public CloseClaw() {
    addRequirements(RobotContainer.m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  RobotContainer.m_Claw.closeClaw();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_Claw.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
