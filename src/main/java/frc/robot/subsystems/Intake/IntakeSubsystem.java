package frc.robot.subsystems.Intake;

import java.util.Optional;
import frc.robot.Robot;
import frc.robot.Constants.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor_left, m_motor_right;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private boolean isIntakeLoaded = false;

    public IntakeSubsystem() {
        this.m_motor_left = new CANSparkMax(Intake.Motors.kUpperMotorLeftID, CANSparkLowLevel.MotorType.kBrushless);
        this.m_motor_right = new CANSparkMax(Intake.Motors.kUpperMotorRightID, CANSparkLowLevel.MotorType.kBrushless);
        m_motor_left.restoreFactoryDefaults();
        // m_motor_left.setSmartCurrentLimit(25);

        m_motor_right.restoreFactoryDefaults();
        // m_motor_right.setSmartCurrentLimit(25);

        // m_motor_left.setSmartCurrentLimit(AllRobot.kAllMotorsLimitInAmpr);
        // m_motor_right.setSmartCurrentLimit(AllRobot.kAllMotorsLimitInAmpr);
    }

    // does this subsystem need configuration? probably? maybe?

    public void pushNote() { // DO NOT USE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // m_motor_left.getPIDController().setReference(Intake.Stats.kPushingNodeInRounds,
        // ControlType.kPosition);
        // m_motor_right.getPIDController().setReference(Intake.Stats.kPushingNodeInRounds
        // * -1, ControlType.kPosition);
    }

    public boolean isLoaded(){
        return isIntakeLoaded;
    }

    public boolean isNotLoaded(){
        return !isIntakeLoaded;
    }

    public void setSpeed(double speed) {
        m_motor_left.set(speed);
        m_motor_right.set(speed * -1);
    }

    public void setVoltage(double voltage) {
        m_motor_left.setVoltage(voltage);
        m_motor_right.setVoltage(-voltage);
    }

    @Override
    public void periodic() {

        int proximity = m_colorSensor.getProximity();
        isIntakeLoaded = proximity > Intake.Stats.kProximitySensorThreshold;
        SmartDashboard.putBoolean("Intake Loaded", isIntakeLoaded);
        SmartDashboard.putNumber("intake speed", m_motor_left.getAppliedOutput());
        SmartDashboard.putNumber("intake voltage", m_motor_left.getBusVoltage());

    }
}