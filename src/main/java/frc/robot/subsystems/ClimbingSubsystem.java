package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.spark.SparkFlex;


public class ClimbingSubsystem extends SubsystemBase {
    
    SparkFlex ClimbMotor;
    
    public ClimbingSubsystem(){
        ClimbMotor = new SparkFlex(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
    }  
        
    public void setClimbMotors(double speed) {
        ClimbMotor.set(speed);
    } 
    
    public void stop() {
        ClimbMotor.set(0);
    }
    
}
