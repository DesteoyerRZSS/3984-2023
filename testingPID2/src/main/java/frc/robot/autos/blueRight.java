package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.arm;
import frc.robot.subsystems.*;

public class blueRight extends SequentialCommandGroup{
    public blueRight(Swerve s_Swerve, Arm s_Arm, Claw s_Claw){
        
        addCommands(
            s_Swerve.alignWNearestRight(s_Swerve.getClosestTag()), 
            new InstantCommand(()->
                s_Arm.GoTo(
                    s_Arm.getAngles(
                        arm.HIGHGOAL[0], 
                        arm.HIGHGOAL[1]
                    )[0], 
                    s_Arm.getAngles(
                        arm.HIGHGOAL[0], 
                        arm.HIGHGOAL[1]
                    )[1]
                )
            )
        );

        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
        Pose2d nextPose = new Pose2d(s_Swerve.getPose().getX() + 0.5, s_Swerve.getPose().getY(), s_Swerve.getPose().getRotation());
        Trajectory t = TrajectoryGenerator.generateTrajectory(
            s_Swerve.getPose(),
            List.of(), 
            nextPose,
            config
        );
        var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            t,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);
        addCommands(swerveControllerCommand, new InstantCommand(()->s_Claw.Outtake()));

    }
}
