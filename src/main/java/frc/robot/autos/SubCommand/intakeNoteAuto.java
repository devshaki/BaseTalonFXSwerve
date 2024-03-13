package frc.robot.autos.SubCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Intake.IntakeNodeCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class intakeNoteAuto extends ParallelCommandGroup {
    public intakeNoteAuto(ArmSubsystem arm, ShooterSubsystem shooters, IntakeSubsystem intake){
    addCommands(new ArmCommand(arm, Arm.Stats.kIntakeAngle),
                        new IntakeNodeCommand(intake, shooters));
    }
}
