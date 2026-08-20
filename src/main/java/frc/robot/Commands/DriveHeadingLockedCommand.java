package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.OperatorConstants;

public class DriveHeadingLockedCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final CommandXboxController controller;

    public DriveHeadingLockedCommand(DriveSubsystem driveSubsystem, CommandXboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.resetHeadingLock();   // optional but recommended
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.kDriveDeadband);
        double ySpeed = MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.kDriveDeadband);

        driveSubsystem.driveHeadingLocked(xSpeed, ySpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // return to normal drive mode
        driveSubsystem.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
