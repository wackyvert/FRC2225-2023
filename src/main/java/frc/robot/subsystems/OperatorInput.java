package frc.robot.subsystems;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ScaleInputs;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class OperatorInput { 
    static boolean chargeStationMode = Robot.chargeMode;
    
    
   public static Joystick Joystick = new Joystick(Constants.OperatorConstants.JOYSTICK_ID);
   
   
        public static double getX() {
          if(chargeStationMode)
          {return 0;}
          return ScaleInputs.scaleInputs(Joystick.getX());
        }
      
        public static double getY() {
          return ScaleInputs.scaleInputs(Joystick.getY());
        }
        public static double getRot(){
          if(chargeStationMode)
          {return 0;}
          return ScaleInputs.scaleInputs(Joystick.getZ(), .3, .1, 4);
        }
        
        
          

}
