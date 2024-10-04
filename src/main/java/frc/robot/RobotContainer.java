package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LauncherConstants;
import frc.robot.autos.Starter;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.devices.Controller.btn;
import frc.robot.devices.OI;
import frc.robot.logger.LoggedPowerDistribution;
import frc.robot.logger.Logger;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.SafeSubsystem;
// import frc.robot.subsystems.Shooter;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// TODO(!!!IMPORTANT!!!): Rebase & merge from the main branch. You will see a 20% drop in CAN
// utilization.

// TODO(!!!IMPORTANT!!!): Run spotless (spotlessApply under VSCode command "Gradle Build") b4 commit

/**
 * Robot structure declaration. Initializes trigger mappings, OI devices, and main stop mechanism.
 */
public class RobotContainer {
  private static boolean m_initialized = false;

  private static final CANLauncher m_launcher = new CANLauncher();

  private static LoggedPowerDistribution powerDist;

  /** Initialize robot container. */
  public static void init() {
    if (m_initialized == true) {
      return;
    }
    m_initialized = true;

    configureOI();
    logMetadata();
    configureBindings();
  }

  /** Initialize driver station specific data. */
  public static void initDS() {
    Logger.recordMetadata("Station", DriverStation.getRawAllianceStation().toString());

    Logger.recordMetadata(
        "Match",
        DriverStation.getEventName()
            + " "
            + DriverStation.getMatchType().toString()
            + " "
            + DriverStation.getMatchNumber());

    Logger.initFilename();
  }

  /** Send metadata to logger. */
  private static void logMetadata() {
    Logger.recordMetadata(
        "Time",
        LocalDateTime.now(ZoneId.of("UTC-8"))
            .format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")));

    Logger.recordMetadata("NT Streaming", String.valueOf(Constants.Config.NTStream));

    Logger.recordMetadata(
        "RoborioSerialNum", RobotBase.isReal() ? System.getenv("serialnum") : "Simulation");

    Logger.recordMetadata("PowerDistributionType", powerDist.getType().name());
  }

  /** Initialize OI devices. */
  private static void configureOI() {
    powerDist = new LoggedPowerDistribution();
    // Stop those ridiculously persistent messages
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** Create button bindings. */
  public static void configureBindings() {
    // Button 'B' (hold) will continuously stop all movement
    new Trigger(
            () -> OI.DRIVER_CONTROLLER.getButton(btn.B) || OI.OPERATOR_CONTROLLER.getButton(btn.B))
        .whileTrue(
            new RunCommand(RobotContainer::stop, SafeSubsystem.getAll())
                // Interrupt incoming commands to ensure stop command takes precedence
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

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
                  // Rumble to indicate odometry has been reset
                  cRumble.schedule();
                }));

    // Button 'Y' (hold) will set drivetrain to x-stance (for stability)
    final var cXStance = new RunCommand(Drivetrain::toXStance, Drivetrain.getInstance());
    OI.DRIVER_CONTROLLER.onTrue(btn.Y, cXStance::schedule);
    OI.DRIVER_CONTROLLER.onFalse(btn.Y, cXStance::cancel);
  }

  private static void configureOperator() {
    // Button 'RB' (hold) Shoots held note using feeder and shooter wheels
    final Command cShoot = Starter.getFullShootCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.RB, cShoot::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.RB, cShoot::cancel);

    // Button 'LB' (hold) Intakes game piece (source) using shooter and feeder wheels
    // final Command cSourceIntake =
    //    Shooter.getInstance()
    //        .getSourceIntakeCommand()
    //        .alongWith(Feeder.getInstance().getFeedDownCommand());
    // OI.OPERATOR_CONTROLLER.onTrue(btn.LB, cSourceIntake::schedule);
    // OI.OPERATOR_CONTROLLER.onFalse(btn.LB, cSourceIntake::cancel);

    // Button 'Y' (hold) Feeds a note upward
    final Command cFeedUp = Feeder.getInstance().getFeedUpCommand();
    OI.OPERATOR_CONTROLLER.onTrue(btn.Y, cFeedUp::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.Y, cFeedUp::cancel);

    // Button 'A' (hold) Feeds a note downward
    // final Command cFeedDown = Feeder.getInstance().getFeedDownCommand();
    // OI.OPERATOR_CONTROLLER.onTrue(btn.X, cFeedDown::schedule);
    // OI.OPERATOR_CONTROLLER.onFalse(btn.X, cFeedDown::cancel);

    /*Create a sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    final Command prepareLaunchCommand =
        new PrepareLaunch(m_launcher)
            .withTimeout(LauncherConstants.kLauncherDelay)
            .andThen(new LaunchNote(m_launcher))
            .handleInterrupt(() -> m_launcher.stop());

    OI.OPERATOR_CONTROLLER.onTrue(btn.A, prepareLaunchCommand::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.A, prepareLaunchCommand::cancel);

    // Set up a binding to run the intake command while the operator is pressing and holding the
    // left Bumper
    final Command intakeCommand = m_launcher.getIntakeCommand();

    OI.OPERATOR_CONTROLLER.onTrue(btn.LB, intakeCommand::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.LB, intakeCommand::cancel);

    /*
     * TODO(climbers): This in fact will not work how you expect it to...
     * The command will schedule when the joystick is first moved and then be active forever.
     * Theres a few things in here that are badly written, I can explain better not in comments.
     * Also, the joystick calls come with a build-in deadband, see the CustomController class.
     *
     * Talk to me for more, or just make the extendClimbers command the default command for the Climbers subsystem so that it's always running. Do this in the constructor.
     * Also, make a private constructor. Even if its empty.
     *l
     * Best, Sid
     */

    final Command cExtendClimbers = Climbers.getInstance().extendClimber();

    // Joystick Y controls climbers
    new Trigger(
            () ->
                Math.abs(OI.OPERATOR_CONTROLLER.getLeftY()) > 0.15
                    || Math.abs(OI.OPERATOR_CONTROLLER.getRightY()) > 0.15)
        .onTrue(new InstantCommand(() -> cExtendClimbers.schedule()));
  }

  /** Calls all subsystem stop methods. Does not stop commands. */
  public static void stop() {
    for (SafeSubsystem e : SafeSubsystem.getAll()) {
      e.stop();
    }
  }
}
