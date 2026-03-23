
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    SparkFlex LeftIntake;
    SparkFlex RightIntake;
    SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(IntakeConstants.kIntakeS, IntakeConstants.kIntakeV);
    PIDController intakePID = new PIDController(IntakeConstants.kIntakeP, IntakeConstants.kIntakeI, IntakeConstants.kIntakeD);
    double desiredSpeedRadS = 0.0;

    public IntakeSubsystem() {
        LeftIntake = new SparkFlex(IntakeConstants.kLeftIntakeMotorCanId, MotorType.kBrushless);
        // LeftIntake.setInverted(true);
        RightIntake = new SparkFlex(IntakeConstants.kRightIntakeMotorCanId, MotorType.kBrushless);
        // 67

        SparkFlexConfig rightConfig = new SparkFlexConfig();
        rightConfig.encoder.velocityConversionFactor((2.0 * Math.PI) / 60.0);
        rightConfig.smartCurrentLimit(80);

        SparkFlexConfig leftConfig = new SparkFlexConfig();
        leftConfig.encoder.velocityConversionFactor((2.0 * Math.PI) / 60.0);
        leftConfig.smartCurrentLimit(80);
        leftConfig.follow(IntakeConstants.kRightIntakeMotorCanId, true);


        LeftIntake.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightIntake.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("setpoint", 0.0);
    }

    public void setIntakeMotors(double speed) {
        desiredSpeedRadS = speed;
    }

    public boolean atSpeed() {
        double currentSpeedRadS = RightIntake.getEncoder().getVelocity();
        return Math.abs(currentSpeedRadS - desiredSpeedRadS) < 5.0; // Tolerance of 5 rad/s
    }

    @Override
    public void periodic() {
        double currentSpeedRadS = RightIntake.getEncoder().getVelocity();
        // desiredSpeedRadS = SmartDashboard.getNumber("setpoint", 0.0);

        double feedforward = intakeFF.calculate(desiredSpeedRadS);
        double feedback = intakePID.calculate(currentSpeedRadS, desiredSpeedRadS);

        SmartDashboard.putNumber("Feedforward Voltage", feedforward);
        SmartDashboard.putNumber("Feedback Voltage", feedback);
        SmartDashboard.putNumber("Current Speed", currentSpeedRadS);
        SmartDashboard.putNumber("Desired Speed", desiredSpeedRadS);
        SmartDashboard.putData("Intake PID", intakePID);

        RightIntake.setVoltage(feedforward + feedback);
    }
}