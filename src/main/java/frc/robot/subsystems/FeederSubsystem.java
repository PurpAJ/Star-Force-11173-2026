package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

    SparkMax Feeder;

    public FeederSubsystem(){
        Feeder = new SparkMax(FeederConstants.kFeederMotorCanId, MotorType.kBrushless);
    }  
        
    public void setFeederMotors(double speed) {
        Feeder.set(speed);
    } 
    
    public void stop() {
        Feeder.set(0);
    }
}