package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class OperatorInput { 
    Joystick leftJoystick = new Joystick(Constants.OperatorConstants.LEFT_JOYSTICK_ID);
    Joystick righJoystick = new Joystick(Constants.OperatorConstants.RIGHT_JOYSTICK_ID);
        public double getX(Joystick joystick) {
          return joystick.getX();
        }
      
        public double getY(Joystick joystick) {
          return joystick.getY();
        }
      
}
