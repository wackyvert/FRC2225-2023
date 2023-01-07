package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    public WPI_TalonFX FL_drive, FR_drive, BL_drive, BR_drive;
    public void initialize(){
        FL_drive = new WPI_TalonFX(Constants.FL_CAN_ID);

        FR_drive = new WPI_TalonFX(Constants.FR_CAN_ID);

        BL_drive = new WPI_TalonFX(Constants.BL_CAN_ID);

        BR_drive = new WPI_TalonFX(Constants.BR_CAN_ID);


        
    }
    @Override
    public void periodic(){

    }
    
}
