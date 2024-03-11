package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShootAmpCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public ShootAmpCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
            DoubleSupplier distance) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.SetRPM(Shooter.Stats.kPutInAmpSpeed);
        intakeSubsystem.setSpeed(Shooter.Stats.kPutInAmpSpeed / 5676);
        // double speed =
        //
        // ShooterHandler.calcShooterAngleAndRPM(distance.getAsDouble()).getFirst();
        // shooterSubsystem.SetRPM(speed);
        // intakeSubsystem.setSpeed(speed / 5676);
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setVoltage(0);
        intakeSubsystem.setVoltage(0);
    }
}
