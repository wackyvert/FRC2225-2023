package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
public class Claw extends SubsystemBase{
    public WPI_TalonFX Arm;
    public WPI_TalonFX Grab;
    public WPI_TalonSRX Pivot;
    public Claw() {
        initialize();
    }

    public void initialize() {
    Arm = new WPI_TalonFX(Constants.Arm_CAN_ID);
    Grab = new WPI_TalonFX(Constants.Grab_CAN_ID);
    Pivot = new WPI_TalonSRX(Constants.Pivot_CAN_ID);
}

public void runPivot(){

Pivot.set(0.3);

}
public void runPivotDown(){
Pivot.set(-0.3);

}
public void stopPivot(){

Pivot.set(0);

}

public void runArmOut(){
Arm.set(.3);

}
public void runArmIn(){
Arm.set(-0.3);

}
public void stopArm(){
Arm.set(0);
}

public void closeClaw(){
Grab.set(.6);

}

public void openClaw(){
Grab.set(-.6);

}
public void stopClaw(){
Grab.set(0);

}

}




 
