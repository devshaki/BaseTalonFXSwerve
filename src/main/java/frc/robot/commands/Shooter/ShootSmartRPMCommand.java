package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShootSmartRPMCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private double m_speed;

    public ShootSmartRPMCommand(ShooterSubsystem shooterSubsystem,
            double rpm) {
        this.shooterSubsystem = shooterSubsystem;
        this.m_speed = rpm;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setSpeed(this.m_speed);
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
    }
}
