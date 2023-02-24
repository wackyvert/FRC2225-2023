package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Intake extends SubsystemBase{
    public WPI_TalonSRX RightBag;
    public WPI_TalonSRX LeftBag;
    public WPI_VictorSPX RightSb;
    public WPI_VictorSPX LeftSb;
    private MotorControllerGroup MoveIntakeGroup, SpinIntakeGroup;
  public Intake(){
     initialize();
  }
public void initialize(){

  RightBag = new WPI_TalonSRX(Constants.RightBag_CAN_ID);
  LeftBag = new WPI_TalonSRX(Constants.LeftBag_CAN_ID);
  RightSb = new WPI_VictorSPX(Constants.RightSb_CAN_ID);
  LeftSb = new WPI_VictorSPX(Constants.LeftSb_CAN_ID);

MoveIntakeGroup = new MotorControllerGroup(LeftSb,RightSb);
SpinIntakeGroup = new MotorControllerGroup(LeftBag,RightBag);

}
public void stopSb(){
RightSb.set(0);
LeftSb.set(0);

}
public void runSb(double speed){
RightSb.set(speed);
LeftSb.set(speed);

}





public void grab(){
runSb(.6);
LeftSb.set(.6);
new WaitCommand(.6);
stopSb();
intakeBagMotors();
}

public void stopBag(){
RightBag.set(0);
LeftBag.set(0);

}

public void runBag(double speed){
RightBag.set(.5);
LeftBag.set(.5);


}



public void intakeBagMotors(){

runBag(.5);
new WaitCommand(.5);
stopBag();

}





} 



