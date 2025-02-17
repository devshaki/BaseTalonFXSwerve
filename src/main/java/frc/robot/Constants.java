package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static class OI {
        public static final int kXboxControllerPort = 0;
        public static final double kXboxcontrollerDrift = 0.1;
    }

    public static class Intake {
        public static class Motors {
            // also the id if there is only one motor
            public static final int kUpperMotorLeftID = 31;
            public static final int kUpperMotorRightID = 34;
        }

        public static class Stats {
            // todo: set the speed needed, and everything in constants honestly
            public static final double kIntakeSpeed = -0.5;
            public static final double kIntakeShootSpeed = -0.6;
            public static final double kIntakeReverseSpeed = 0.4;
            public static final double kPushingNodeInRounds = 1;
            public static final double kShooterSpeed = -0.1;
            public static final double kProximitySensorThreshold = 55;
        }
    }

    public static class Shooter {
        public static class Motors {
            public static final int ShooterMotorLeftID = 32;
            public static final int ShooterMotorRightID = 33;

        }

        public static class Stats {
            // todo: set the speed needed
            public static final double kIShooterSpeed = 0.0;
            public static final double kPutInAmpSpeed = 0.0;
        }

        public static class PID {
            public static final double kP = 0.85;
            public static final double kI = 0.00;
            public static final double kD = 0;
        }
    }

    public static class Arm {
        public static class Motors {
            public static final int kLeftMotorID = 22;
            public static final int kRightMotorID = 21;
        }

        public static class Stats {
            public static final double kLimitAngle = 90;
            // todo put actual angles and then it will do the thing
            public static final double encoderOffset = 156;
            public static final double gearRatio = 1 / 25.0;
            public static final double ampAngle = 110;
            public static final double speakerAngle = 3 ;
            public static final double speakerAngleCloseAngle = 15 ;
            public static final double speakerAngleFar = 20;
            public static final double speakerAngleFarAngle = 15;
            public static final double driveAngle = 40;
            public static final double kThreashold = 0;
            public static final double kIntakeAngle = 0;
            public static final double climbAngle = 0;

            public static final double maxVoltage = 20;
        }

        public static class Encoders {
            public static final int kLeftEncoderID = 0;
            public static final int kRightEncoderID = 0;
        }

        public static class Pid {
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.02;
            public static final double kF = 0.65; // volts
        }

    }

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.75); // TODO: This must be tuned to specific
                                                                             // robot
        public static final double wheelBase = Units.inchesToMeters(21.75); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 60;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.433571986927912; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 17;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((0.459473 * 360) - 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((0.957275 * 360));
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 19;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((0.566162 * 360) - 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 15;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((0.194824 * 360) - 180);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 20.0;
        public static final double kPYController = 20;
        public static final double kPThetaController = 4.0;

        public static final double kPThetaControllerDrive = 0.1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
