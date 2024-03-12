package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.Intake.IntakeCommand;

import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

import frc.robot.autos.SingleNoteAuto;

public class DoubleNoteAuto extends SequentialCommandGroup {
    public DoubleNoteAuto(Swerve s_Swerve, ArmSubsystem arm, ShooterSubsystem shooters, IntakeSubsystem intake){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond/2.0,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory Trajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(Units.inchesToMeters(40), 0)),
                new Pose2d(Units.inchesToMeters(40), 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand SwerveControllerCommand =
            new SwerveControllerCommand(
                Trajectory,
                s_Swerve::getPoseInvertedGyro,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())), // SET INITIAL POSITIONs
            new SingleNoteAuto(arm, shooters, intake,Constants.Arm.Stats.speakerAngle), // SHOOT LOADED NOTE
            new ParallelCommandGroup(
                SwerveControllerCommand, // DRIVE TO NOTE
                new intakeNoteAuto(arm, shooters, intake)).withTimeout(2), // INTAKE NOTE
            new SingleNoteAuto(arm, shooters, intake,Constants.Arm.Stats.speakerAngleFar)
        );
    }
}