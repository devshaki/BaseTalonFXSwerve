package frc.robot.autos.Left;

import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
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
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeNodeCommand;
import frc.robot.commands.Shooter.ShootSmartRPMCommand;
import frc.robot.commands.Shooter.ShootVoltageCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class LeftTripleNoteAuto extends SequentialCommandGroup {
    public LeftTripleNoteAuto(Swerve s_Swerve, ArmSubsystem arm, ShooterSubsystem shooters, IntakeSubsystem intake, boolean inverted) {
        Trajectory TrajectoryB = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(4.0,0.0)),
                new Pose2d(6.8, -0.10, new Rotation2d(0)),
                autoUtils.trajectoryConfig());

        var TrajecotryCommandB = autoUtils.CommandFromTrajectory(TrajectoryB, s_Swerve);

        addCommands(
                new LeftDoubleNoteAuto(s_Swerve,arm,shooters,intake, inverted),
                new InstantCommand(() -> s_Swerve.setPose(TrajectoryB.getInitialPose())), // SET INITIAL POSITIONs
                TrajecotryCommandB.alongWith(new ParallelCommandGroup(
                    new ArmCommand(arm, Arm.Stats.kIntakeAngle-2.5),
                    new ShootSmartRPMCommand(shooters, 4500),
                    new IntakeCommand(intake, Intake.Stats.kIntakeSpeed)
                ).onlyWhile(() -> !TrajecotryCommandB.isFinished())),
                new SingleNoteAuto(arm, shooters, intake, Constants.Arm.Stats.speakerAngleFarAngle) // SHOOT LOADED NOTE
        );
    }
}