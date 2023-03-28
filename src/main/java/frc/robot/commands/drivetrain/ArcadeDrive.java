package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OperatorInput;
import frc.robot.RobotContainer;
import frc.robot.ScaleInputs;

public class ArcadeDrive extends CommandBase {
  boolean xbox = false;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive() {
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mDrivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
if (!xbox) {
    RobotContainer.mDrivetrain.hDrive(ScaleInputs.scaleInputs(OperatorInput.getY()), ScaleInputs.scaleInputs(OperatorInput.getRot(), .35, .15, .7)*.7, ScaleInputs.scaleInputs(OperatorInput.getRot(),.3,.15,2));
  }
  else{
    //RobotContainer.mDrivetrain.hDrive(ScaleInputs.scaleInputs(OperatorInput.getcontrollerY()), ScaleInputs.scaleInputs(OperatorInput.getcontrollerX())*.7, ScaleInputs.scaleInputs(OperatorInput.getcontrollerX()));
  }
}
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  
}