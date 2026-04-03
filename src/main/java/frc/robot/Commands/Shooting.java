 package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Shooting extends Command {
    IntakeSubsystem m_IntakeSubsystem;
    FeederSubsystem m_FeederSubsystem;
    double m_setpoint;

    
  public Shooting(IntakeSubsystem IntakeSubsystem, FeederSubsystem FeederSubsystem, double setpoint) {
    m_IntakeSubsystem = IntakeSubsystem;
    m_FeederSubsystem = FeederSubsystem;

    addRequirements(m_IntakeSubsystem);
    addRequirements(m_FeederSubsystem);
    m_setpoint = setpoint;
  }

  @Override
  public void initialize() {
    m_IntakeSubsystem.setIntakeMotors(m_setpoint);
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
  }
    
}