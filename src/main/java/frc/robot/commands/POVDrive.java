// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OperatorInput;

public class POVDrive extends CommandBase {
  /** Creates a new POVDrive. */
  public POVDrive() {
    addRequirements(RobotContainer.mDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  int pov;
  double rot;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().getDefaultCommand(RobotContainer.mDrivetrain).cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pov=OperatorInput.getPOVDrive();
    //chatgpt helped me write this. i did not want to type it all and to my surprise it did it no issue!
    rot = OperatorInput.getSecondRot();
    if(pov==0){
      RobotContainer.mDrivetrain.hDrive(0.3, 0, 0);
    } else if(pov==45){
      RobotContainer.mDrivetrain.hDrive(0.3, 0, -0.2);
    } else if(pov==90){
      RobotContainer.mDrivetrain.hDrive(0, 0, -0.2);
    } else if(pov==135){
      RobotContainer.mDrivetrain.hDrive(-0.3, 0, -0.2);
    } else if(pov==180){
      RobotContainer.mDrivetrain.hDrive(-0.3, 0, 0);
    } else if(pov==225){
      RobotContainer.mDrivetrain.hDrive(-0.3, 0, 0.2);
    } else if(pov==270){
      RobotContainer.mDrivetrain.hDrive(0, 0, 0.2);
    } else if(pov==315){
      RobotContainer.mDrivetrain.hDrive(0.3, 0, 0.2);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.mDrivetrain, new ArcadeDrive());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
