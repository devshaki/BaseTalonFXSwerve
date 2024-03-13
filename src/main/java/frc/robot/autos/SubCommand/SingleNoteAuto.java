package frc.robot.autos.SubCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shooter.ShootSmartRPMCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.HoldCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;

public class SingleNoteAuto extends SequentialCommandGroup {
    public SingleNoteAuto(ArmSubsystem arm, ShooterSubsystem shooters, IntakeSubsystem intake, double ShooterAngle) {
        addCommands(
                new ParallelCommandGroup(
                        new ShootSmartRPMCommand(shooters, 4500),
                        new ArmCommand(arm, ShooterAngle),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new IntakeCommand(intake, Intake.Stats.kIntakeSpeed)))
                        .withTimeout(2));
    }
}