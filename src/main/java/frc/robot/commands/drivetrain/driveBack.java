// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class driveBack extends CommandBase {
  public double averageEncoders;
  
  /** Creates a new driveBack. */
  public driveBack() {
    addRequirements(RobotContainer.mDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    averageEncoders=RobotContainer.mDrivetrain.getAverageEncoders();
    RobotContainer.mDrivetrain.arcadeDrive(-.3, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mDrivetrain.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(averageEncoders>Units.feetToMeters(3)){
      return true;
    }
    else {return false;}
  }
}
