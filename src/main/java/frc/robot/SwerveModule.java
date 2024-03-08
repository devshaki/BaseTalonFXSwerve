package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private CANSparkFlex mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS,
            Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert.value == 0 ? true : false);
        mDriveMotor.setIdleMode(
                Constants.Swerve.driveNeutralMode == NeutralModeValue.Brake ? IdleMode.kBrake : IdleMode.kCoast);
        mDriveMotor.getEncoder().setVelocityConversionFactor(1 / (Constants.Swerve.driveGearRatio * 60));
        mDriveMotor.getEncoder().setPositionConversionFactor(1 / Constants.Swerve.driveGearRatio);

        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveCurrentLimit);
        mDriveMotor.getPIDController().setP(Constants.Swerve.driveKP);
        mDriveMotor.getPIDController().setD(Constants.Swerve.driveKD);
        mDriveMotor.getPIDController().setI(Constants.Swerve.driveKI);
        mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);

        mDriveMotor.getEncoder().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(driveDutyCycle.Output);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                    Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.getPIDController().setReference(driveVelocity.Velocity, ControlType.kVelocity, 0,
                    driveVelocity.FeedForward);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getEncoder().getVelocity(), Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getEncoder().getPosition(),
                        Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
    }
}