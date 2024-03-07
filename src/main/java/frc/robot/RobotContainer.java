package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.devices.Controller.btn;
import frc.robot.devices.OI;
import frc.robot.logger.LoggedPowerDistribution;
import frc.robot.logger.Logger;
import frc.robot.subsystems.*;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

/**
 * Robot structure declaration. Initializes trigger mappings, OI devices, and main stop mechanism.
 */
public class RobotContainer {
  private static boolean m_initialized = false;

  private static LoggedPowerDistribution powerDist;

  /** Initialize robot container. */
  public static void init() {
    if (m_initialized == true) {
      return;
    }
    m_initialized = true;

    configureOI();
    configureMetadata();
    configureBindings();
  }

  /** Send metadata to logger. */
  private static void configureMetadata() {
    Logger.recordMetadata(
        "RoborioSerialNum", RobotBase.isReal() ? System.getenv("serialnum") : "Simulation");
    Logger.recordMetadata(
        "BuildDate",
        LocalDateTime.now(ZoneId.of("UTC-8"))
            .format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")));
    Logger.recordMetadata("PowerDistributionType", powerDist.getType().name());
    Logger.recordMetadata("NT Streaming", Constants.Config.NTStream ? "true" : "false");
  }

  /** Initialize OI devices. */
  private static void configureOI() {
    powerDist = new LoggedPowerDistribution();
    DriverStation.silenceJoystickConnectionWarning(
        true); // Stop those ridiculously persistent messages
  }

  /** Create button bindings. */
  private static void configureBindings() {
    // Button 'B' (hold) will continuously stop all movement
    new Trigger(
            () -> OI.DRIVER_CONTROLLER.getButton(btn.B) || OI.OPERATOR_CONTROLLER.getButton(btn.B))
        .whileTrue(
            new RunCommand(
                    RobotContainer::stop, SafeSubsystem.getAll() // Requires all subsystems
                    )
                .withInterruptBehavior(
                    InterruptionBehavior
                        .kCancelIncoming)); // Interrupt incoming commands to ensure stop command
    // takes precedence

    configureDriver();
    configureOperator();
  }

  private static void configureDriver() {
    // Button 'X' (debounced 0.25s) will reset heading
    final var cRumble = OI.DRIVER_CONTROLLER.getRumbleCommand(0.5, 0.5, 0.25);
    new Trigger(() -> OI.DRIVER_CONTROLLER.getButton(btn.X))
        .debounce(0.25) // Wait 0.25s to avoid accidental press
        .onTrue(
            new InstantCommand(
                () -> {
                  // Reset odometry so that forward is away from the driver station
                  Drivetrain.resetOdometry(
                      new Pose2d(Drivetrain.getPose().getTranslation(), new Rotation2d(0)));
                  cRumble.schedule(); // Rumble to indicate odometry has been reset
                }));

    // Button 'Y' (hold) will set drivetrain to x-stance (for stability)
    final var cXStance = new RunCommand(Drivetrain::toXStance, Drivetrain.getInstance());
    OI.DRIVER_CONTROLLER.onTrue(btn.Y, cXStance::schedule);
    OI.DRIVER_CONTROLLER.onFalse(btn.Y, cXStance::cancel);
  }

  private static void configureOperator() {
    // Button 'RB' (hold) Shoots held note
    final Command cShoot = Shooter.getInstance().getShootCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.RB, cShoot::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.RB, cShoot::cancel);

    // Button 'A' (hold) Spits held note for amp scoring
    final Command cSpit = Shooter.getInstance().getSpitCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.A, cSpit::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.A, cSpit::cancel);

    // Button 'LB' (hold) Intakes game piece using shooter wheels
    final Command cSourceIntake = Shooter.getInstance().getSourceIntakeCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.LB, cSourceIntake::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.LB, cSourceIntake::cancel);

    // Button 'X' (hold) Feeds note through from the intake to the shooter
    final Command cFeed = Feeder.getInstance().getFeedCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.X, cFeed::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.X, cFeed::cancel);

    // Button 'Y' (hold) Deeply intakes a note from the source
    final Command cDeepIntake = Feeder.getInstance().getDeepIntakeCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.Y, cDeepIntake::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.Y, cDeepIntake::cancel);
  }

  /** Calls all subsystem stop methods. Does not stop commands. */
  public static void stop() {
    for (SafeSubsystem e : SafeSubsystem.getAll()) {
      e.stop();
    }
  }
}
