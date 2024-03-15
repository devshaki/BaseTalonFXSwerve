package frc.robot.autos.Center;

import frc.robot.Constants;
import frc.robot.autos.SubCommand.SingleNoteAuto;
import frc.robot.autos.SubCommand.intakeNoteAuto;
import frc.robot.subsystems.Swerve;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

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

public class CenterTripleNoteAuto extends SequentialCommandGroup {
    public CenterTripleNoteAuto(Swerve s_Swerve, ArmSubsystem arm, ShooterSubsystem shooters, IntakeSubsystem intake) {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory Trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(45), 0, new Rotation2d(0)),
                List.of(new Translation2d(Units.inchesToMeters(20), Units.inchesToMeters(35))),
                new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(65), new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0,
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand SwerveControllerCommand = new SwerveControllerCommand(
                Trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        Trajectory Trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(60), new Rotation2d(0)),
                List.of(),
                new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(0), new Rotation2d(0)),
                config);

        SwerveControllerCommand SwerveControllerCommand2 = new SwerveControllerCommand(
                Trajectory2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
                new CenterDoubleNoteAuto(s_Swerve, arm, shooters, intake),
                new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())), // SET INITIAL

                // SwerveControllerCommand,// POSITIONs
                SwerveControllerCommand.alongWith(
                        new intakeNoteAuto(arm, shooters, intake).onlyWhile(() -> !SwerveControllerCommand.isFinished())), // INTAKE NOTE
                new InstantCommand(() -> s_Swerve.zeroModules()),

                new InstantCommand(() -> s_Swerve.setPose(Trajectory2.getInitialPose())), // SET INITIAL
                SwerveControllerCommand2,
                new InstantCommand(() -> s_Swerve.zeroModules()),
                new SingleNoteAuto(arm, shooters, intake, Constants.Arm.Stats.speakerAngleFar));
    }
}