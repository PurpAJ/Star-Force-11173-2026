package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

  // ---------------------------------------------------------
  // 🚗 Drive / Swerve Constants
  // ---------------------------------------------------------
  public static final class DriveConstants {

    // Maximum allowed speeds (not the motor's physical max)
    public static final double kMaxSpeedMetersPerSecond = 5.4;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // rad/sec

    // Robot geometry
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    public static final double kWheelBase = Units.inchesToMeters(23.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Rear Left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Rear Right

    // Module angular offsets (REV MAXSwerve defaults)
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 9;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 3;

    public static final double kHeadingP = 0.0;
    public static final double kHeadingI = 0.0;
    public static final double kHeadingD = 0.0;

    // navX yaw direction (false = normal)
    public static final boolean kGyroReversed = false;
  }

  

  private static Pose2d makeTarget(double x, double y) {
    return new Pose2d(new Translation2d(x, y), Rotation2d.kZero);
  }

  public static final class Targets {
    public static final double FIELD_LENGTH = 16.5405;
    public static final double FIELD_WIDTH = 8.0695;

    // Boundaries for zones of the field, used to determine which target to aim at
    public static final double BLUE_ALLIANCE_LINE_X = 4.4;
    public static final double RED_ALLIANCE_LINE_X = FIELD_LENGTH - 4.4;
    public static final double CENTER_LINE_Y = FIELD_WIDTH / 2;

    // Targets for the shooter to aim at
    public static final Pose2d BLUE_HUB = makeTarget(4.625, CENTER_LINE_Y);
    public static final Pose2d RED_HUB = makeTarget(FIELD_LENGTH - 4.625, CENTER_LINE_Y);
    public static final Pose2d BLUE_PASS_OUTPOST = makeTarget(0, 1);
    public static final Pose2d BLUE_PASS_DEPOT = makeTarget(0, FIELD_WIDTH - 1);
    public static final Pose2d RED_PASS_OUTPOST = makeTarget(FIELD_LENGTH - 0, FIELD_WIDTH - 1);
    public static final Pose2d RED_PASS_DEPOT = makeTarget(FIELD_LENGTH - 0, 1);
  }

  public enum AimMode {
    HUB,
    PASS,
  }

  // ---------------------------------------------------------
  // 🟦 Intake
  // ---------------------------------------------------------
  public static final class IntakeConstants {
    public static final int kLeftIntakeMotorCanId = 13;
    public static final int kRightIntakeMotorCanId = 15;

    public static final double kIntakeVelocity = 450;
    public static final double kShootTowerVelocity = 480;
    public static final double kFeedNeutralVelocity = 550;

    public static final double kIntakeP = 0.02;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0005;
    public static final double kIntakeS = 0.175;
    public static final double kIntakeV = 0.017;
  }

  public static final class FeederConstants {
    public static final int kFeederMotorCanId = 12;
    public static final double kFeederMotorCurrentLimit = 40;
    public static double kFeederFowardSpeed = 0.6;
    public static double kFeederReverseSpeed = -0.6;
  }

  // ---------------------------------------------------------
  // 🎮 Operator Controls
  // ---------------------------------------------------------
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  // ---------------------------------------------------------
  //  Module Constants (MAXSwerve)
  // ---------------------------------------------------------
  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;

    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // Gear reduction: (45 * 22) / (pinion * 15)
    public static final double kDrivingMotorReduction = (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  // ---------------------------------------------------------
  // 🤖 Autonomous
  // ---------------------------------------------------------
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared);
  }

  // ---------------------------------------------------------
  // ⚡ NEO Motor Specs
  // ---------------------------------------------------------
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
