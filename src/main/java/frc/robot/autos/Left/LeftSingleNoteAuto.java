package frc.robot.autos.Left;

import frc.robot.Constants;
import frc.robot.autos.autoUtils;
import frc.robot.autos.rotateAuto;
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

public class LeftSingleNoteAuto extends SequentialCommandGroup {
    public LeftSingleNoteAuto(Swerve s_Swerve, ArmSubsystem arm, ShooterSubsystem shooters, IntakeSubsystem intake, boolean inverted) {
        Trajectory TrajectoryA = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(1, 0, new Rotation2d(0)),
                autoUtils.trajectoryConfig());

        var TrajecotryCommandA = autoUtils.CommandFromTrajectory(TrajectoryA, s_Swerve);

        addCommands(
                new InstantCommand(() -> s_Swerve.zeroHeading()), // SET INITIAL POSITIONs
                new rotateAuto(s_Swerve, Rotation2d.fromDegrees(-45 * (inverted ? -1 : 1))),
                new SingleNoteAuto(arm, shooters, intake, Constants.Arm.Stats.speakerAngleCloseAngle), // SHOOT LOADED NOTE
                new rotateAuto(s_Swerve, Rotation2d.fromDegrees(0)),

                // 
                
                new InstantCommand(() -> s_Swerve.setPose(TrajectoryA.getInitialPose())), // SET INITIAL POSITIONs
                TrajecotryCommandA.alongWith(new intakeNoteAuto(arm, shooters, intake).onlyWhile(() -> !TrajecotryCommandA.isFinished()))
        );
    }
}