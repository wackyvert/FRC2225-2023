// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class AlignVision extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  PhotonCamera camera = new PhotonCamera("OV5647");
  double visionRotationSpeed;
  PIDController turnController = new PIDController(.10,0,0);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignVision(Drivetrain mDrivetrain) {
    m_subsystem = mDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().getDefaultCommand(RobotContainer.mDrivetrain).cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
        // Calculate angular turn power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        visionRotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
       // If we have no targets, stay still.
       visionRotationSpeed = 0;
    } 
    DriverStation.reportWarning("Speed = "+visionRotationSpeed, true);
    RobotContainer.mDrivetrain.arcadeDrive(0, visionRotationSpeed/2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.mDrivetrain.stopAll();
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.mDrivetrain, new ArcadeDrive());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
