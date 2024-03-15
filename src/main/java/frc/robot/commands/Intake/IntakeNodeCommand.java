package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class IntakeNodeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public IntakeNodeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(intakeSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setSpeed(Intake.Stats.kIntakeSpeed);
        shooterSubsystem.setVoltage(Intake.Stats.kShooterSpeed * 12);
    }

    @Override
    public boolean isFinished() {
        if(intakeSubsystem.isLoaded()){
            end(true);
        }
        return intakeSubsystem.isLoaded();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0);
        shooterSubsystem.setVoltage(0);
    }
}
