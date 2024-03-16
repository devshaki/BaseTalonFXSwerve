package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.autos.autoUtils;

public class testAuto extends SequentialCommandGroup {
    public testAuto(Swerve s_Swerve){
        
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(3, 0, Rotation2d.fromDegrees(0))
                    ),
                autoUtils.trajectoryConfig());

        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
            // autoUtils.CommandFromTrajectory(exampleTrajectory, s_Swerve)
            new rotateAuto(s_Swerve,Rotation2d.fromDegrees(90))
        );
    }
}