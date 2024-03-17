package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor_left, m_motor_right;
    private final SparkPIDController m_left_pid;
    private final SparkPIDController m_right_pid;

    public ShooterSubsystem() {
        this.m_motor_left = new CANSparkMax(Shooter.Motors.ShooterMotorLeftID, CANSparkLowLevel.MotorType.kBrushless);
        this.m_motor_right = new CANSparkMax(Shooter.Motors.ShooterMotorRightID, CANSparkLowLevel.MotorType.kBrushless);
        m_motor_left.restoreFactoryDefaults();
        m_motor_right.restoreFactoryDefaults();
        this.m_left_pid = m_motor_left.getPIDController();
        this.m_right_pid = m_motor_right.getPIDController();

        m_left_pid.setP(Shooter.PID.kP);
        m_left_pid.setI(Shooter.PID.kI);
        m_left_pid.setD(Shooter.PID.kD);
        m_left_pid.setSmartMotionMaxVelocity(5000, 0);
        m_left_pid.setSmartMotionMaxAccel(1, 0);

        m_right_pid.setP(Shooter.PID.kP);
        m_right_pid.setI(Shooter.PID.kI);
        m_right_pid.setD(Shooter.PID.kD);
    }

    public void SetRPM(double speed) {
        // m_left_pid.setReference(speed, ControlType.kVelocity);
        // m_right_pid.setReference(-speed, ControlType.kVelocity);

    }

    public void setVoltage(double voltage) {
        // m_motor_right.setVoltage(voltage);
        // m_motor_left.setVoltage(-voltage);
    }

    public void setSpeed(double rpm) {
        // m_motor_right.getPIDController().setReference(rpm, ControlType.kVelocity);
        // m_motor_left.getPIDController().setReference(-rpm, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter speed", m_motor_right.getEncoder().getVelocity());
        SmartDashboard.putNumber("shooter voltage", m_motor_right.getBusVoltage());
    }
}