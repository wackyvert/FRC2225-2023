package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.ScaleInputs;

public class OperatorInput { 
    static boolean chargeStationMode = false;
   public static Joystick leftJoystick = new Joystick(Constants.OperatorConstants.LEFT_JOYSTICK_ID);
   public static Joystick righJoystick = new Joystick(Constants.OperatorConstants.RIGHT_JOYSTICK_ID);
        public static double getX() {
          if(chargeStationMode)
          {return 0;}
          return ScaleInputs.scaleInputs(leftJoystick.getX());
        }
      
        public static double getY() {
          return ScaleInputs.scaleInputs(leftJoystick.getY());
        }
        public static double getRot(){
          if(chargeStationMode)
          {return 0;}
          return ScaleInputs.scaleInputs(leftJoystick.getZ(), .3, .1, 4);
        }
      
}
