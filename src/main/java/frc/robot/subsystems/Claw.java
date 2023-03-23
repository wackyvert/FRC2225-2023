package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Claw extends SubsystemBase{
    public WPI_TalonFX Arm;
    public WPI_TalonFX Grab;
    public WPI_TalonSRX Pivot;
    public double PivotEncoder;
    public PIDController armPidController = new PIDController(.019511, 0, .00435);
    ArmFeedforward ff = new ArmFeedforward(Constants.armkS, Constants.armkG, Constants.armkV, Constants.armkA);
    public Claw() {
        initialize();
    }

    public void initialize() {

    Arm = new WPI_TalonFX(Constants.Arm_CAN_ID);
    Grab = new WPI_TalonFX(Constants.Grab_CAN_ID);
    Pivot = new WPI_TalonSRX(Constants.Pivot_CAN_ID);
    Pivot.setNeutralMode(NeutralMode.Brake);
    Arm.setNeutralMode(NeutralMode.Brake);
}
public void periodic(){
PivotEncoder=Pivot.getSelectedSensorPosition()/2048/100;
}
public double getPivotEncoder() {
    return PivotEncoder;
}
public void ZeroPivot(){
    Pivot.setSelectedSensorPosition(0);
    
}
public void runPivot(){

Pivot.set(0.5);
}
public void pivotPosition(){
    Pivot.setVoltage(ff.calculate(Units.degreesToRadians(90), Pivot.getSelectedSensorVelocity()));
}
public void runPivotDown(){
Pivot.set(-0.5);

}
public void stopPivot(){

Pivot.set(0);

}

public void runArmOut(){
Arm.set(.65);

}
public void runArmIn(){
Arm.set(-0.45);

}
public void stopArm(){
Arm.set(0);
}

public void closeClaw(){
Grab.set(.7);

}

public void openClaw(){
Grab.set(-.7);

}
public void stopClaw(){
Grab.set(0);

}

}




 
