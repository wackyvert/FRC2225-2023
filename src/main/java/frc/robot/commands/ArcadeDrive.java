package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OperatorInput;
import frc.robot.RobotContainer;
import frc.robot.ScaleInputs;

public class ArcadeDrive extends CommandBase {
  
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mDrivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {RobotContainer.mDrivetrain.hDrive(OperatorInput.getY(), OperatorInput.getX(), OperatorInput.getRot());}
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  
}