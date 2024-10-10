package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SwerveModule.SwerveModuleConstants;
import frc.robot.util.TunableNumber;

// Global constants. Should not include functional code.
public class Constants {
  // Information for testing and robot configuration that must be updated consistenly.
  public static class Config {
    // Set true to stream log file data to NetworkTables (takes up bandwith and processing time, but useful for concurrent running and visualization).
    public static final boolean NTStream = false;

    // Pigeon mount offsets - REMEMBER TO UPDATE, or configure through PhoenixTuner.
    public static final MountPoseConfigs kPigeonMountPose =
      new MountPoseConfigs().withMountPoseYaw(0).withMountPosePitch(0).withMountPoseRoll(0);
  }

  // Ports and properties of non-motor devices.
  public static class DeviceConstants {
    public static final byte kDriverPort = 0;
    public static final byte kOperatorPort = 1;
    public static final byte kPigeonID = 2;
  }

  // Physical parts of the robot, such as gearboxes or wheel diameters.
  public static class PhysicalConstants {
    // NEO V1.1 nominal voltage.
    public static final int kNEOMaxVoltage = 12;

    // NEO V1.1 general current limit (40A-60A is advised).
    public static final int kNEOCurrentLimit = 40;

    // NEO V1.1 empirical free speed.
    public static final double kSwerveDriveMaxRPS = 5680d / 60d;

    // Ratio of drive wheels to motors for SDS L3 modules with 16T drive pinions.
    public static final double kSwerveDriveMechToSens = 1d / 5.355;

    // Billet wheels.
    public static final double kSwerveWheelCircumferenceMeters = 0.099 * Math.PI;

    // Ratio of feeder wheels to motor.
    public static final double kFeederMechToSens = 11d / 32d;

    // Ratio of shooter wheels to motors.
    public static final double kShooterMechToSens = 1;
  }

  // Data relating to the entire drivetrain.
  public static class DriveConstants {
    // Upper bound drivetrain constraints.
    // Competition speed: 80% of theoretical max.
    public static final double kMaxLinearVelMetersPerSecond =
      PhysConstants.kSwerveDriveMaxRPS
        * PhysConstants.kSwerveDriveMechToSens
        * PhysConstants.kSwerveWheelCircumferenceMeters
        * 1.0; // Speed during competition was 80%. Reduced to 60% for Sunset Showdown.

    // ω = velocity / radius.
    public static final double kMaxTurnVelRadiansPerSecond =
      kMaxLinearVelMetersPerSecond / Constants.SwerveConstants.kDriveBaseRadius;

    // To avoid brownouts and overpowering.
    public static final double kModuleAzimuthMaxVoltage = 0.65 * PhysConstants.kNEOMaxVoltage;
    public static final double kModuleDriveMaxVoltage = 0.95 * PhysConstants.kNEOMaxVoltage;
    public static final int kModuleAzimuthCurrentLimit = 30;

    // Multipliers for all teleop driving.
    public static final double kTeleopSpeedMult = 1;
    // Set maximum teleop turn speed to 1.5 rotations/s.
    public static final double kTeleopTurnMult = 9.5 / kMaxTurnVelRadiansPerSecond;

    // Update rate for drivetrain, default 20 ms. 8-64 ms is best for NEO hall sensor.
    public static final int kPeriodMs = 10;
  }

  public static class AmperConstants {
    // CAN ID for motor controller.
    public static final int kAmperID = 4;

    // Current limit for amper wheel.
    public static final int kAmperCurrentLimit = 60;

    // Speeds for wheels when intaking and scoring. Scoring speed is negative to run the wheels in reverse.
    public static final double kAmperIntakeSpeed = -0.4;
    public static final double kAmperScoreSpeed = 0.4;
    public static final double kAmperStallSpeed = -0.1;
  }

  public static class KitBotConstants {
    // CAN IDs for motor controllers.
    public static final int kFeederID = 5;
    public static final int kLauncherID = 6;

    // Current limit for launcher and feed wheels.
    public static final int kLauncherCurrentLimit = 80;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Launch speeds are negative to run the wheels in reverse.
    public static final double kLauncherSpeed = -1;
    public static final double kFeederSpeed = -1;
    public static final double kIntakeLauncherSpeed = 1;
    public static final double kIntakeFeederSpeed = .3;

    // Delay for starting the feeder wheel to allow the launcher wheel to spin up.
    public static final double kFeederDelay = 1;
  }

  /*
  public static class ShooterConstants {
    public static final byte kTopShooterMotorID = 61;
    public static final byte kBottomShooterMotorID = 62;

    public static final double kSourceIntakeVolts = 8; // Competition season speed was 9; changed to 8 for Sunset Showdown
    public static final double kShootVolts = -8; // Competition season speed was -10; changed to -8 for Sunset Showdown
  }

  public static class FeederConsts {
    public static final byte kFeedMotorID = 3;

    public static final double kFeedUpVolts = -10; // Competition season speed was -10
    public static final double kFeedDownVolts = 1; // Competition season speed was 1
  }
  */

  public static class ClimberConstants {
    public static final byte kLeftClimberID = 7; // Originally 6; swapped to 7 with KitBot.
    public static final byte kRightClimberID = 8; // Originally 7; swapped to 8 with KitBot.

    public static final double kClimberVolts = -12; // Competition season speed was -12.

    // Current limit for climber motors.
    public static final int kClimberCurrentLimit = 25;
  }

  // Data for each individual swerve module.
  public static class SwerveConstants {
    // Gains for module velocity error -> voltage.
    public static final TunableNumber kDriveS = new TunableNumber("S", 0.1, "Module Drive");
    public static final TunableNumber kDriveP = new TunableNumber("P", 2, "Module Drive");

    // Whether azimuth motor is inverted, use for Mk4i's.
    public static final boolean kAzimuthInverted = true;

    public static final SwerveModuleConstants
      kSwerve_fl =
        new SwerveModuleConstants(
          "SwerveFL",
          // Azimuth gains (kS, kP, kD)
          0.1, // Originally 0.1
          0.1, // Originally 0.095
          0.0005, // Originally 0.0006
          -0.678466, // CANcoder offset
          11, // Driving motor CAN ID.
          12, // Turning motor CAN ID.
          13, // CANCoder CAN ID.
          new Translation2d(-0.212461, -0.263261) // 0.212471, 0.263271
        ),
      kSwerve_fr =
        new SwerveModuleConstants(
          "SwerveFR",
          // Azimuth gains (kS, kP, kD)
          0.1, // Originally 0.092
          0.1, // Originally 0.092
          0.0005, // Originally 0.00065
          -0.183593, // CANcoder offset
          21, // Driving motor CAN ID.
          22, // Turning motor CAN ID.
          23, // CANCoder CAN ID.
          new Translation2d(-0.212461, 0.263261) // 0.212471, -0.263271
        ),
      kSwerve_bl =
        new SwerveModuleConstants(
          "SwerveBL",
          // Azimuth gains (kS, kP, kD)
          0.1, // Originally 0.08
          0.1, // Originally 0.105
          0.0005, // Originally 0.0004
          -0.103759, // CANcoder offset
          31, // Driving motor CAN ID.
          32, // Turning motor CAN ID.
          33, // CANCoder CAN ID.
          new Translation2d(0.212461, -0.263261) // -0.212471, 0.263271
        ),
      kSwerve_br =
        new SwerveModuleConstants(
          "SwerveBR",
          // Azimuth gains (kS, kP, kD)
          0.1, // Originally 0.092
          0.1, // Originally 0.09
          0.0005, // Originally 0.00065
          -0.499023, // CANcoder offset
          41, // Driving motor CAN ID.
          42, // Turning motor CAN ID.
          43, // CANCoder CAN ID.
          new Translation2d(0.212461, 0.263261) // -0.212471, -0.263271
        );

    // Drive base radius for angular velocity calcs (use swerve module farthest from COR).
    public static final double kDriveBaseRadius =
      kSwerve_bl.location.getDistance(new Translation2d());
  }

  public static class AutoConstants {
    // TODO (dev/user): Ensure that drivetrain acceleration limits are strong.
    // Upper bound drivetrain accelerations for path following and pose targeting.
    public static final double kMaxLinearAccelMetersPerSecondSquared =
      DriveConstants.kMaxLinearVelMetersPerSecond;
    public static final double kMaxTurnAccelRadiansPerSecondSquared =
      DriveConstants.kMaxTurnVelRadiansPerSecond;

    // TODO(user): Tune pathfinding PID gains (currently at default gains).
    // Gains for drivetrain position error -> velocity.
    public static final TunableNumber kTranslateP = new TunableNumber("P", 5, "Robot Translation");
    public static final TunableNumber kTranslateD = new TunableNumber("D", 0, "Robot Translation");
  }
}
