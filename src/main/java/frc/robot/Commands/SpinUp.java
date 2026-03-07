package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinUp extends Command {
    IntakeSubsystem m_IntakeSubsystem;

    
  public SpinUp(IntakeSubsystem IntakeSubsystem) {
    m_IntakeSubsystem = IntakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  @Override
  public void initialize() {
    m_IntakeSubsystem.setIntakeMotors(IntakeConstants.kLeftIntakeInSpeed);
    
    
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
    m_IntakeSubsystem.stop();
  }
    
}
