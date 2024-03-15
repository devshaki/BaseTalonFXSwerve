package frc.robot.autos.Center;

import frc.robot.Constants;
import frc.robot.autos.autoUtils;
import frc.robot.autos.rotateAuto;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeCommand;

import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class CenterTripleNoteAutoRotate extends SequentialCommandGroup {
    public CenterTripleNoteAutoRotate(Swerve s_Swerve, ArmSubsystem arm, ShooterSubsystem shooters,
            IntakeSubsystem intake) {
        Trajectory Trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(65), 0.4, new Rotation2d(0))),
                autoUtils.trajectoryConfig);

        var SwerveControllerCommand1 = autoUtils.CommandFromTrajectory(Trajectory1, s_Swerve);

        Trajectory Trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(Units.inchesToMeters(65), 0.4, new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(0), 0.0, new Rotation2d(0))),
                autoUtils.trajectoryConfig);

        var SwerveControllerCommand2 = autoUtils.CommandFromTrajectory(Trajectory2, s_Swerve);


        addCommands(
                new CenterDoubleNoteAuto(s_Swerve, arm, shooters, intake),
                new rotateAuto(s_Swerve, Rotation2d.fromDegrees(-90)),

                new InstantCommand(() -> s_Swerve.setPose(Trajectory1.getInitialPose())), // SET INITIAL
                SwerveControllerCommand1.alongWith(
                        new intakeNoteAuto(arm, shooters, intake).onlyWhile(
                                () -> !SwerveControllerCommand1.isFinished())), // INTAKE NOTE
                new InstantCommand(() -> s_Swerve.zeroModules()),

                SwerveControllerCommand2, // INTAKE NOTE
                new InstantCommand(() -> s_Swerve.zeroModules()),
                
                new rotateAuto(s_Swerve, Rotation2d.fromDegrees(90)),
                new SingleNoteAuto(arm, shooters, intake, Constants.Arm.Stats.speakerAngleFar)
        // new rotateAuto(s_Swerve, Rotation2d.fromDegrees(-90)),
        // new SingleNoteAuto(arm, shooters, intake,
        // Constants.Arm.Stats.speakerAngleFar)
        );
    }
}