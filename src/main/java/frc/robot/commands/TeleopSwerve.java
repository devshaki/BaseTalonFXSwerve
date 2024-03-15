package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private DoubleSupplier speedModSupplier;
    private PIDController thetaController;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, DoubleSupplier speedModSupplier) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.speedModSupplier = speedModSupplier;

        s_Swerve.targetHeading = -999;

        thetaController = new PIDController(Constants.AutoConstants.kPThetaControllerDrive, 0, 0);
        thetaController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if(Math.abs(rotationVal) > 0){
            s_Swerve.targetHeading = -999;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed * MathUtil.interpolate(1, 0.05, speedModSupplier.getAsDouble())), 
            s_Swerve.targetHeading == -999 ? rotationVal * Constants.Swerve.maxAngularVelocity : thetaController.calculate(s_Swerve.getPoseInvertedGyro().getRotation().getDegrees()), 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}