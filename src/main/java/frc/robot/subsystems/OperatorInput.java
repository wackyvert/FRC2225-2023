package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class OperatorInput { 
    static Joystick leftJoystick = new Joystick(Constants.OperatorConstants.LEFT_JOYSTICK_ID);
    static Joystick righJoystick = new Joystick(Constants.OperatorConstants.RIGHT_JOYSTICK_ID);
        public static double getX() {
          return leftJoystick.getX();
        }
      
        public static double getY() {
          return leftJoystick.getY();
        }
        public static double getRot(){
          return leftJoystick.getZ();
        }
      
}
