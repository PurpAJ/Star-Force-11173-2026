 package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Shooting extends Command {
    IntakeSubsystem m_IntakeSubsystem;
    FeederSubsystem m_FeederSubsystem;

    
  public Shooting(IntakeSubsystem IntakeSubsystem, FeederSubsystem FeederSubsystem) {
    m_IntakeSubsystem = IntakeSubsystem;
    m_FeederSubsystem = FeederSubsystem;

    addRequirements(m_IntakeSubsystem);
    addRequirements(m_FeederSubsystem);
  }

  @Override
  public void initialize() {
    m_IntakeSubsystem.setIntakeMotors(ShootingConstants.ShootingSpeed);
    m_FeederSubsystem.setFeederMotors(FeederConstants.kFeederReverseSpeed);
    
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
    m_FeederSubsystem.stop();
  }
    
}