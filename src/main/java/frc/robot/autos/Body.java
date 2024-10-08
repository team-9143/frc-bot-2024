package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// If selected, will determine movement configurations depending on starting position.
public enum Body {
  Escape,
  None;

  public Command getCommand(StartPose startPose) {
    switch (this) {
      case Escape:
        if (startPose == StartPose.Wing) {
          // Move forward 6 meters from starting position,
          var path =
            Pathing.generateDirectPath(
              StartPose.Wing.getRawPose(),
              StartPose.Wing.getRawPose().plus(new Transform2d(6, 0, new Rotation2d()))
            );
          // Path will be flipped automatically during this call, no need to flip above.
          return Pathing.getHolonomicFollowPathCommand(path);
        }

        // IMP!!! Make sure that pathplanner path names match start poses.
        return Pathing.getHolonomicFollowPathCommand(Pathing.loadPath(startPose.toString()));

      default:
        return new InstantCommand();
    }
  }
}
