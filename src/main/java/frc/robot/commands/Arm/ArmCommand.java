package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmCommand extends Command {
    private ArmSubsystem armSubsystem;
    private double angle;

    public ArmCommand(ArmSubsystem armSubsystem, double angle) {
        this.armSubsystem = armSubsystem;
        this.angle = angle;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.SetAngle(angle);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Arm Correct", Math.abs(armSubsystem.getAngle() - angle) < Arm.Stats.kThreashold);
        return Math.abs(armSubsystem.getAngle() - angle) < Arm.Stats.kThreashold;
    }

    @Override
    public void execute() {
        armSubsystem.execute();
    }

    @Override
    public void end(boolean interrupted) {  
        armSubsystem.setVoltage(0);
        armSubsystem.setMaxVoltage(Arm.Stats.maxVoltage);
    }
}
