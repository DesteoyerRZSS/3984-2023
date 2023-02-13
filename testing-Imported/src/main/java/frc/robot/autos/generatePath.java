package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

// Code that generates path
/* 1. Initialize 3 positions relative to april tags, left right straight
 * 2. Look for april tag, get location on the field.
 * 3. Auto generate path to the position based on user input / auto chooser
 * 4. Raise arm and move to o position
 */
public class generatePath extends SequentialCommandGroup {
    public static void generatePath(Swerve s_Swerve){
        var result = Constants.Swerve.camera.getLatestResult();
        if (result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d pose = target.getBestCameraToTarget();
            Translation3d poseTrans = pose.getTranslation();
            
            //Trajectory TrajectoryToScore = new TrajectoryGenerator.generateTrajectory();
        }
        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    }
    public static enum State {
        Score1, Score2, Score3, path;
    }
}
