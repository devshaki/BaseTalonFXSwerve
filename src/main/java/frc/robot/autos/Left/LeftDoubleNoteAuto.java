package frc.robot.autos.Left;

import frc.robot.Constants;
import frc.robot.autos.SubCommand.SingleNoteAuto;
import frc.robot.autos.SubCommand.intakeNoteAuto;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeCommand;

import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class LeftDoubleNoteAuto extends SequentialCommandGroup {
    public LeftDoubleNoteAuto(Swerve s_Swerve, ArmSubsystem arm, ShooterSubsystem shooters, IntakeSubsystem intake) {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond / 2,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        Trajectory TrajectoryA = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(Units.inchesToMeters(0),Units.inchesToMeters(-55))),
                new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(-60), new Rotation2d(0)),
                config);


        SwerveControllerCommand SwerveControllerCommandA = new SwerveControllerCommand(
                TrajectoryA,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        Trajectory TrajectoryB = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(-65), Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(Units.inchesToMeters(5), 0, new Rotation2d(0)),
                config);


        SwerveControllerCommand SwerveControllerCommandB = new SwerveControllerCommand(
                TrajectoryB,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
                new InstantCommand(() -> s_Swerve.setPose(TrajectoryA.getInitialPose())), // SET INITIAL POSITIONs
                new SingleNoteAuto(arm, shooters, intake, Constants.Arm.Stats.speakerAngle), // SHOOT LOADED NOTE
                SwerveControllerCommandA.alongWith(
                    new intakeNoteAuto(arm, shooters, intake).onlyWhile(() -> !SwerveControllerCommandA.isFinished())),
                new InstantCommand(() -> s_Swerve.zeroModules()), // INTAKE NOTE
                

                new InstantCommand(() -> s_Swerve.setPose(TrajectoryB.getInitialPose())), // SET INITIAL POSITIONs
                SwerveControllerCommandB,
                new InstantCommand(() -> s_Swerve.zeroModules()),
                new SingleNoteAuto(arm, shooters, intake, Constants.Arm.Stats.speakerAngleFar));
    }
}