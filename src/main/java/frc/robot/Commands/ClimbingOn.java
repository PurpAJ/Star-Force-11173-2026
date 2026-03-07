package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimbingSubsystem;

public class ClimbingOn extends Command {
    ClimbingSubsystem m_ClimbingSubsystem;

    public ClimbingOn(ClimbingSubsystem ClimbingSubsystem) {
        m_ClimbingSubsystem = ClimbingSubsystem;

        addRequirements(m_ClimbingSubsystem);
    }

    @Override
    public void initialize() {
        m_ClimbingSubsystem.setClimbMotors(ClimberConstants.kClimberOnSpeed);
        
    }

    @Override
    public void execute() {
  
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_ClimbingSubsystem.stop();
    }
    
}
