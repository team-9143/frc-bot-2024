package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeviceConsts;
import frc.robot.Constants.FieldConsts;
import frc.robot.devices.Limelight;
import frc.robot.subsystems.Drivetrain;

public class AutoAlign extends Command{
    private static Limelight limelight;


    @Override
    public void initialize() {
        super.initialize();
    }

    public static double[] getDistanceFromAngle(Limelight limelight){
        double yangle = limelight.getTy();
        double xangle = limelight.getTx();
        double yangleradians = (yangle + DeviceConsts.limelightAngle) * (3.14 / 180);
        double ydistance = FieldConsts.speakerAprilTagY - DeviceConsts.limelightHeight /Math.tan(yangleradians);
        double xdistance = ydistance/Math.tan(xangle);
        return new double[] {xdistance, ydistance};
    }

    public static Pose2d getAprilTagPose(){
        return new Pose2d(getDistanceFromAngle(limelight)[0], getDistanceFromAngle(limelight)[1], new Rotation2d(limelight.getTx() * (3.14 / 180)));
    }

    @Override
    public void execute() {
        double yangle = limelight.getTy();
        double xangle = limelight.getTx();
        double ydistance = FieldConsts.speakerAprilTagY/Math.tan(yangle); 
        double xdistance = ydistance/Math.tan(xangle);
        double yspeed = ydistance > 1 ? ydistance: ydistance > 15 ? 15 : 0;
        double xspeed =  xdistance > 1 ? xdistance: xdistance > 15 ? 15 : 0;
        Drivetrain.getInstance().driveRobotRelativeVelocity(new ChassisSpeeds(xspeed, yspeed, -xangle));
        super.execute();
    }

    @Override
    public void cancel() {
        super.cancel();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}