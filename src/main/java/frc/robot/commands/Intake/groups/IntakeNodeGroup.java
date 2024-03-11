package frc.robot.commands.Intake.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Intake.IntakeNodeCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class IntakeNodeGroup extends ParallelCommandGroup {
    ArmSubsystem armSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public IntakeNodeGroup(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addCommands(new IntakeNodeCommand(intakeSubsystem, shooterSubsystem),
                new ArmCommand(armSubsystem, Arm.Stats.kIntakeAngle));

        addRequirements(armSubsystem, intakeSubsystem, shooterSubsystem);
    }
}
