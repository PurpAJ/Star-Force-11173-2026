package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class Feeder extends Command {
    FeederSubsystem m_FeederSubsystem;

    
  public Feeder(FeederSubsystem FeederSubsystem) {
    m_FeederSubsystem = FeederSubsystem;

    addRequirements(m_FeederSubsystem);
  }

  @Override
  public void initialize() {
    m_FeederSubsystem.setFeederMotors(FeederConstants.kFeederFowardSpeed);
    
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
    m_FeederSubsystem.stop();
  }
 
}
