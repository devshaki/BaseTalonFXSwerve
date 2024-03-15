package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class IntakeGroup extends ParallelCommandGroup {
    public IntakeGroup(ArmSubsystem m_ArmSubsystem, IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem) {
        addCommands(new ParallelCommandGroup(new ArmCommand(m_ArmSubsystem, Arm.Stats.kIntakeAngle),
                        new IntakeNodeCommand(m_IntakeSubsystem, m_ShooterSubsystem)));
    }
}
