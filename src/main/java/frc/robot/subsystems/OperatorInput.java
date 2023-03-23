package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ScaleInputs;
public class OperatorInput { 
    static boolean chargeStationMode = Robot.chargeMode;
    
    
   public static Joystick joystick = new Joystick(Constants.OperatorConstants.RIGHTJOYSTICK_ID);
   public static Joystick joystick2 = new Joystick(Constants.OperatorConstants.LEFTJOYSTICK_ID);
   public static XboxController xboxController = new XboxController(Constants.OperatorConstants.XBOX_CONTROLLER_ID);
   
        public static double getX() {
          if(chargeStationMode)
          {return 0;}
          return ScaleInputs.scaleInputs(joystick.getX());
        }
        public static double getcontrollerX() { //should be left stick horizontal axis
          return ScaleInputs.scaleInputs(xboxController.getRawAxis(0));
        }
        public static double getcontrollerY() { //should be left stick vertical axis
          return ScaleInputs.scaleInputs(xboxController.getRawAxis(1));
        }
        public static double getcontrollerRot() { //should be right stick horizontal axis
          return ScaleInputs.scaleInputs(xboxController.getRawAxis(4));
        }
        public static double getY() {
          return ScaleInputs.scaleInputs(joystick.getY());
        }
        public static double getRot(){
          if(chargeStationMode)
          {return 0;}
          return ScaleInputs.scaleInputs(joystick.getZ(), .2, .1, 4);
        }
        public static double getSecondRot(){
          if(chargeStationMode)
          {return 0;}
          return ScaleInputs.scaleInputs(joystick2.getZ(), .2, .1, 4)*.3;
        }
        public static int getPOVDrive(){
          int pov=joystick2.getPOV();
          return pov;
          
        }
        
        
          

}
