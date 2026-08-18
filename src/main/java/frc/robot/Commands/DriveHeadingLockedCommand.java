package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionAlignConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveHeadingLockedCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final CommandXboxController controller;

    public DriveHeadingLockedCommand(
        DriveSubsystem driveSubsystem,
        CommandXboxController controller
    ) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = -controller.getLeftY();
        double ySpeed = -controller.getLeftX();

        driveSubsystem.driveHeadingLocked(xSpeed, ySpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}