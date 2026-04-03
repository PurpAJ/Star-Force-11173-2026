package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinUp extends Command {
    IntakeSubsystem m_IntakeSubsystem;

    double m_setpoint;

    
  public SpinUp(IntakeSubsystem IntakeSubsystem, double setpoint) {
    m_IntakeSubsystem = IntakeSubsystem;
    addRequirements(m_IntakeSubsystem);
    m_setpoint = setpoint;
  }

  @Override
  public void initialize() {
    m_IntakeSubsystem.setIntakeMotors(m_setpoint);
  }

  @Override
  public void execute() {
 
  }

  @Override
  public boolean isFinished() {
    return m_IntakeSubsystem.atSpeed();
  }

  @Override
  public void end(boolean interrupted) {
  }
    
}
