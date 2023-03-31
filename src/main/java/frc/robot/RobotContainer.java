// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants.AutoConstants;
import frc.robot.Constants.OperatorConstants.DriveConstants;
import frc.robot.commands.arm.ArmIn;
import frc.robot.commands.arm.ArmOut;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.drivetrain.AlignVision;
import frc.robot.commands.drivetrain.DropBreaks;
import frc.robot.commands.drivetrain.POVDrive;
import frc.robot.commands.drivetrain.RaiseBreaks;
import frc.robot.commands.drivetrain.StopBreaks;
import frc.robot.commands.intake.DropIntake;
import frc.robot.commands.intake.IntakeSpin;
import frc.robot.commands.intake.IntakeSpinIn;
import frc.robot.commands.intake.RaiseIntake;
import frc.robot.commands.intake.StartIntakeAuto;
import frc.robot.commands.intake.StopDrop;
import frc.robot.commands.intake.stopIntake;
import frc.robot.commands.pivot.TurnPivot;
import frc.robot.commands.pivot.TurnPivotDown;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OperatorInput;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of Pthe robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain mDrivetrain = new Drivetrain();
  public static final Claw m_Claw = new Claw();
  public static final Intake mIntake = new Intake();
  public static final OperatorInput oi = new OperatorInput();
  public static boolean povControl = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick joystick = new CommandJoystick(Constants.OperatorConstants.RIGHTJOYSTICK_ID);
  CommandJoystick secondJoystick = new CommandJoystick(Constants.OperatorConstants.LEFTJOYSTICK_ID);
  XboxController xboxController = new XboxController(Constants.OperatorConstants.XBOX_CONTROLLER_ID);
  
  
  
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  Trigger button1 = joystick.button(1).whileTrue(new IntakeSpinIn());
  Trigger button2 = joystick.button(2).whileTrue(new IntakeSpin());
  Trigger button3 = joystick.button(3).onTrue(new SequentialCommandGroup(new DropIntake(), new WaitCommand(.27), new StopDrop()));
  Trigger button4 = joystick.button(4).onTrue(new SequentialCommandGroup(new RaiseIntake(), new WaitCommand(.27), new StopDrop()));
  Trigger button14 = joystick.button(14).whileTrue(new AlignVision(mDrivetrain));
  Trigger secondButton1 = secondJoystick.button(1).whileTrue(new ArmOut());
  Trigger secondButton2 = secondJoystick.button(2).whileTrue(new ArmIn());
  Trigger secondButton3 = secondJoystick.button(3).whileTrue(new CloseClaw());
  Trigger secondButton4 = secondJoystick.button(4).whileTrue(new OpenClaw());
  Trigger secondButton14 = secondJoystick.button(14).whileTrue(new POVDrive());
  Trigger secondButton15 =secondJoystick.button(15).onTrue(new SequentialCommandGroup(new DropBreaks(), new WaitCommand(2.2), new StopBreaks()));
  Trigger secondButton16 = secondJoystick.button(16).onTrue(new SequentialCommandGroup(new RaiseBreaks(), new WaitCommand(2.2), new StopBreaks()));
  Trigger yAxisPositive = secondJoystick.axisGreaterThan(1, .5).whileTrue(new TurnPivot());
  Trigger yAxisNegative = secondJoystick.axisLessThan(1,-.5).whileTrue(new TurnPivotDown());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory chosenTrajectory) {

    // An example command will be run in autonomous
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);

    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(Units.feetToMeters(6), 0), new Translation2d(Units.feetToMeters(10), 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.feetToMeters(16), 0, new Rotation2d(0)),
        // Pass config
        config);
    chosenTrajectory = exampleTrajectory;

    RamseteCommand ramseteCommand = new RamseteCommand(
        chosenTrajectory,
        mDrivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        mDrivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        mDrivetrain::tankDriveVolts,
        mDrivetrain);

    // Reset odometry to the starting pose of the trajectory.
    mDrivetrain.resetOdometry(chosenTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return  new SequentialCommandGroup(new StartIntakeAuto(), new WaitCommand(1), new stopIntake(), ramseteCommand.andThen(() -> mDrivetrain.tankDriveVolts(0, 0)));
    
  }
}
