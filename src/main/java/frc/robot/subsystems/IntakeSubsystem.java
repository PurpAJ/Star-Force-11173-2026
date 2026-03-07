 
package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkFlex;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    
        // SparkFlex LeftIntake;
        SparkFlex RightIntake;
    
        public IntakeSubsystem(){
        // LeftIntake = new SparkFlex(IntakeConstants.kLeftIntakeMotorCanId, MotorType.kBrushless);
        // LeftIntake.setInverted(true);
        RightIntake = new SparkFlex(IntakeConstants.kRightIntakeMotorCanId, MotorType.kBrushless);
//67
    }  
        
    public void setIntakeMotors(double speed) {
        // LeftIntake.set(speed);
        RightIntake.set(speed);
    } 
    
    public void stop() {
        // LeftIntake.set(0);
        RightIntake.set(0);
    }
}