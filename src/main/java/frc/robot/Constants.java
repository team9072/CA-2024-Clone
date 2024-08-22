package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class RobotConstants {
    public static final double k_width = 28; // Inches
    public static final double k_length = 28; // Inches
    public static final int kPdhId = 1;
  }

  public static class Intake {
    // Motors
    public static final int kIntakeMotorId = 12;
    public static final int kPivotMotorId = 13;

    // DIO
    public static final int k_pivotEncoderId = 0;
    public static final int k_intakeLimitSwitchId = 2;

    // Absolute encoder offset
    public static final double k_pivotEncoderOffset = 0.166842; // Straight up, sketchy to reset to "up"

    // Pivot set point angles
    public static final double k_pivotAngleGround = 60;
    public static final double k_pivotAngleSource = 190;
    public static final double k_pivotAngleAmp = k_pivotAngleSource;
    public static final double k_pivotAngleStow = 275;

    // Intake speeds
    public static final double k_intakeSpeed = 0.7;
    public static final double k_ejectSpeed = -0.45;
    public static final double k_feedShooterSpeed = -0.5;
  }

  // PCM
  public static final int kPCMId = 0;
  public static final int kIntakeSolenoidForwardId = 2;

  // DIO

  // Shooter
  public static final int kShooterLeftMotorId = 10;
  public static final int kShooterRightMotorId = 11;

  public static final double kShooterP = 0.00005;
  public static final double kShooterI = 0.0;
  public static final double kShooterD = 0.0;
  public static final double kShooterFF = 0.0002;

  public static final double kShooterMinOutput = 0;
  public static final double kShooterMaxOutput = 1;

  // Climber
  public static final int kClimberLeftMotorId = 14;
  public static final int kClimberRightMotorId = 15;
  public static final double kClimberClimbSpeed = 600.0; // RPM
  public static final double kClimberReleaseSpeed = -600.0; // RPM

  public static final double kClimberGearRatio = 1.0 / 12.0;

  public static final double kClimberP = 0.001;
  public static final double kClimberI = 0.0;
  public static final double kClimberD = 0.0;
  public static final double kClimberMinOutput = -0.5;

  public static final double kClimberMaxOutput = 0.5;

  // Old diff drivetrain constants
  @Deprecated
  public static class Drive {
    public static final double kP = 0.085;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.01;
    public static final double kV = 2.6;

    public static final int kFLMotorId = 8;
    public static final int kBLMotorId = 7;
    public static final int kFRMotorId = 6;
    public static final int kBRMotorId = 5;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds

    public static final double kMaxSpeedMetersPerSecond = 4.8 * 1.3;// speeds
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kAutoAimMaxAngularSpeed = Units.radiansToDegrees(540.00);
    public static final double kAutoAimMaxAngularAccel = Units.radiansToDegrees(720.00);

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration (28x28)
    public static final double kTrackWidth = Units.inchesToMeters(RobotConstants.k_width);
    // Distance between centers of right and left wheels on robot (24*24)
    public static final double kWheelBase = Units.inchesToMeters(RobotConstants.k_length); // 18.5?
    // Distance from center to furthest wheel (*diagonal*)
    public static final double kCenterToWheel = Units.inchesToMeters(Math.sqrt(121 + 121)); // 11^2 + 11^2 PT
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kDiagonalMeters = Units.inchesToMeters(33.941);

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kRearRightTurningCanId = 9;

    public static final boolean kGyroReversed = true;
    public static final double kGyroAdjustment = 0.0;

    public static final PIDConstants kTranslationPID = new PIDConstants(5, 0); // Translation PID constants
    public static final PIDConstants kRotationPID = new PIDConstants(5, 0, 0); // Rotation PID constants

    // PathFlowerConfig for PathPlanner's AutoBuilder
    public static final HolonomicPathFollowerConfig AutoPathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0), // Translation PID constants
        new PIDConstants(5, 0, 0), // Rotation PID constants
        10.0, // Max module speed, in m/s
        DriveConstants.kCenterToWheel, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
    );
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 16;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kFreeSpeedRpm = 5676;
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.365 / 1); // 3.365
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedMps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedMps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 60; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static class Field {
    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);

    // TODO: Add left and right subwoofer starting poses
    public static final Pose2d redCenterPose2d = new Pose2d(15.19, 5.50, new Rotation2d(Units.degreesToRadians(180.0)));
    public static final Pose2d blueCenterPose2d = new Pose2d(1.27, 5.50, new Rotation2d(0));
  }

  public static class LEDs {
    public static final int k_PWMId = 0;
    public static final int k_totalLength = 300;
  }
}
