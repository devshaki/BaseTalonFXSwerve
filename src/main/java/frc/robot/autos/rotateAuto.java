package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeSubsystem;


public class rotateAuto extends Command {
        private Swerve swerveSubsystem;
        private PIDController thetaController;

        public rotateAuto(Swerve swerveSubsystem, Rotation2d targetAngle) {
            this.swerveSubsystem = swerveSubsystem;
            this.swerveSubsystem.targetHeading = targetAngle.getDegrees();
            thetaController = new PIDController(0.01, 0, 0);
            thetaController.enableContinuousInput(-180, 180);
            thetaController.setSetpoint(targetAngle.getDegrees());
            addRequirements(swerveSubsystem);
        }

        @Override   
        public void initialize() {
            swerveSubsystem.zeroModules();
        }

        @Override
        public boolean isFinished() {
            SmartDashboard.putNumber("Robot Target Heading", thetaController.getSetpoint());
            swerveSubsystem.drive(new Translation2d(0,0), thetaController.calculate(swerveSubsystem.getHeading().getDegrees()), true, false);
            return (Math.abs(swerveSubsystem.getPoseInvertedGyro().getRotation().getDegrees() - this.swerveSubsystem.targetHeading) < 2.5);
        }

        @Override
        public void end(boolean interrupted) {
            swerveSubsystem.zeroModules();
            swerveSubsystem.targetHeading = -999;
        }
    }