package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;

public class DriveHeadingLockedCommand extends Command {

    private final DriveSubsystem drive;
    private final CommandXboxController controller;

    public DriveHeadingLockedCommand(DriveSubsystem drive, CommandXboxController controller) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.kDriveDeadband);
        double y = MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.kDriveDeadband);

        drive.driveHeadingLocked(x, y);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
