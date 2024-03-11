package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class IntakeVoltageCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    // private ShooterSubsystem shooterSubsystem;
    private double m_voltage;

    public IntakeVoltageCommand(IntakeSubsystem intakeSubsystem,
            double voltage) {
        // this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.m_voltage = voltage;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        // shooterSubsystem.setVoltage(this.m_voltage);
        intakeSubsystem.setSpeed(1);
        // double speed =
        //
        // ShooterHandler.calcShooterAngleA`ndRPM(distance.getAsDouble()).getFirst();
        // shooterSubsystem.SetRPM(speed);
        // intakeSubsystem.setSpeed(speed / 5676);
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        // shooterSubsystem.setVoltage(0);
        intakeSubsystem.setVoltage(0);
    }
}
