// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmIn;
import frc.robot.commands.ArmOut;

import frc.robot.commands.ChargeSwitch;
import frc.robot.commands.CloseClaw;

import frc.robot.commands.Grab;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.TurnPivot;
import frc.robot.commands.TurnPivotDown;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OperatorInput;

import org.opencv.osgi.OpenCVNativeLoader;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of Pthe robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain mDrivetrain = new Drivetrain();
  public static final Claw m_Claw = new Claw();
  public static final Intake mIntake = new Intake();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  Joystick joystick = new Joystick(Constants.OperatorConstants.JOYSTICK_ID);
  Trigger button14 = new JoystickButton(joystick, 14);
  Trigger button13 = new JoystickButton(joystick, 13);
  Trigger button2 = new JoystickButton(joystick, 2);
  Trigger button3 = new JoystickButton(joystick, 3);
  Trigger button4 = new JoystickButton(joystick, 4);
  Trigger button1 = new JoystickButton(joystick, 1);
  Trigger button5 = new JoystickButton(joystick, 5);
  Trigger button6 = new JoystickButton(joystick, 6);



  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    button14.onTrue(new ChargeSwitch());
    button13.onTrue(new Grab());
    button2.whileTrue(new CloseClaw());
    button1.whileTrue(new OpenClaw());
    button3.whileTrue(new ArmOut());
    button4.whileTrue(new ArmIn());
    button5.whileTrue(new TurnPivotDown());
    button6.whileTrue(new TurnPivot());
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
