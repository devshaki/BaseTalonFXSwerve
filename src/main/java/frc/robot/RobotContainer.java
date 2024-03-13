package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.autos.*;
import frc.robot.autos.Center.CenterTripleNoteAuto;
import frc.robot.autos.Left.LeftDoubleNoteAuto;
import frc.robot.commands.*;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.HoldCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeNodeCommand;
import frc.robot.commands.Shooter.ShootSmartRPMCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(frc.robot.Constants.OI.kXboxControllerPort + 1);

    // thee arm
    private final ArmSubsystem m_ArmSubsystem;

    private final IntakeSubsystem m_IntakeSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
    private final OI oi;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.oi = new OI();
        this.m_IntakeSubsystem = new IntakeSubsystem();
        this.m_ShooterSubsystem = new ShooterSubsystem();
        this.m_ArmSubsystem = new ArmSubsystem();
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        /* Intake Button */
        oi.commandXboxController.a()
                .whileTrue(new ParallelCommandGroup(new ArmCommand(m_ArmSubsystem, Arm.Stats.kIntakeAngle),
                        new IntakeNodeCommand(m_IntakeSubsystem, m_ShooterSubsystem)));// Intake command group
        /* Shooter Buttons */
        oi.commandXboxController.rightBumper()
                .whileTrue(new ParallelCommandGroup(new ShootSmartRPMCommand(m_ShooterSubsystem, 4500),
                        new ArmCommand(m_ArmSubsystem, Arm.Stats.speakerAngle)));
        oi.commandXboxController.x().whileTrue(new IntakeCommand(m_IntakeSubsystem, Intake.Stats.kIntakeSpeed));
        /* Default command */
        m_ArmSubsystem.setDefaultCommand(new HoldCommand(m_ArmSubsystem));
        /* Reversed intake */
        oi.commandXboxController.y().whileTrue(new IntakeCommand(m_IntakeSubsystem, Intake.Stats.kIntakeReverseSpeed));

        // oi.commandXboxController.b().whileTrue(new ArmCommand(m_ArmSubsystem,
        // Arm.Stats.kIntakeAngle));

        // oi.commandXboxController.rightBumper()
        // .whileTrue(new ParallelCommandGroup(new
        // ShootSmartRPMCommand(m_ShooterSubsystem, 4500),
        // new ArmCommand(m_ArmSubsystem, Arm.Stats.speakerAngle)));

        // oi.commandXboxController.leftBumper()
        // .whileTrue(new ArmCommand(m_ArmSubsystem, Arm.Stats.ampAngle));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous

        return new SequentialCommandGroup(new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)))),
                new ArmCommand(m_ArmSubsystem, 20).withTimeout(1),
                new LeftDoubleNoteAuto(s_Swerve, m_ArmSubsystem, m_ShooterSubsystem, m_IntakeSubsystem)).withTimeout(15),
                new InstantCommand(() -> s_Swerve.zeroModules()));
        // new SingleNoteAuto(m_ArmSubsystem, m_ShooterSubsystem,
        // m_IntakeSubsystem,Constants.Arm.Stats.speakerAngleFar));
    }
}
