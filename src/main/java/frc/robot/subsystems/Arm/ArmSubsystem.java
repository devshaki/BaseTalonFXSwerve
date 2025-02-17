// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.lang.annotation.Target;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.commands.Arm.HoldCommand;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor_left, m_motor_right;
    private final DutyCycleEncoder m_encoder;
    private double maxVoltage = 12;

    private PIDController pidController = new PIDController(Arm.Pid.kP, Arm.Pid.kD, Arm.Pid.kI);
    
    public ArmSubsystem() {
        m_motor_left = new CANSparkMax(Arm.Motors.kLeftMotorID, MotorType.kBrushless);
        m_motor_right = new CANSparkMax(Arm.Motors.kRightMotorID, MotorType.kBrushless);
        m_motor_left.restoreFactoryDefaults();
        m_motor_right.restoreFactoryDefaults();
        m_motor_left.burnFlash();
        m_motor_right.burnFlash();

        m_encoder = new DutyCycleEncoder(Arm.Encoders.kLeftEncoderID);
        pidController.enableContinuousInput(0,360);
        // pidController.setTolerance(1);
        this.setAngleToidle();

        // m_motor_left.setSmartCurrentLimit(AllRobot.kAllMotorsLimitInAmpr);
        // m_motor_right.setSmartCurrentLimit(AllRobot.kAllMotorsLimitInAmpr);
        setAngleToCurrent();
        setDefaultCommand(new HoldCommand(this));
        
        m_motor_left.setIdleMode(IdleMode.kBrake);
        m_motor_right.setIdleMode(IdleMode.kBrake);
        
        m_motor_right.follow(m_motor_left, true);
        SmartDashboard.putNumber("arm_voltage", 0);
        SmartDashboard.putNumber("arm_setpoint", pidController.getSetpoint());
        SmartDashboard.putNumber("Arm kP", pidController.getP());
        SmartDashboard.putNumber("Arm kI", pidController.getI());
        SmartDashboard.putNumber("Arm kD", pidController.getD());
    }

    public void setVoltage(double voltage) {
        // voltage = MathUtil.clamp(voltage, -12, 12);
        SmartDashboard.putNumber("arm_wanted_voltage", voltage);
        m_motor_left.setVoltage(-voltage - Arm.Pid.kF);
        m_motor_right.setVoltage(voltage + Arm.Pid.kF);
        // m_motor_left.setVoltage(-Arm.Pid.kF);
        // m_motor_right.setVoltage(Arm.Pid.kF);

    }

    public void SetAngle(double TargetAngle) {
        pidController.setSetpoint(TargetAngle);
    }

    public void setAngleToCurrent() {
        pidController.setSetpoint(getAngle());
    }

    public void setAngleToidle() {
        pidController.setSetpoint(30);
    }
    

    public double getAngle() {
        return 360 - (m_encoder.getAbsolutePosition() * 360.0)-Arm.Stats.encoderOffset;
    }

    public double getMaxVoltage(){
        return maxVoltage;
    }

    public void setMaxVoltage(double voltage){
        this.maxVoltage = voltage;
    }

    public void execute() {
        double voltage = 0;
        if(Math.abs(maxVoltage) > 12)
            voltage = pidController.calculate(getAngle());
        else
            voltage = Math.min(Math.max(-maxVoltage,pidController.calculate(getAngle())), maxVoltage);
        setVoltage(voltage);
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Arm Angle [Drivers]", 90 - getAngle());
        SmartDashboard.putNumber("arm_left_output", m_motor_left.getAppliedOutput());
        SmartDashboard.putNumber("arm_setpoint", pidController.getSetpoint());
        double[] armdata = {pidController.getSetpoint(), getAngle()}; 
        SmartDashboard.putNumberArray("Arm Data",armdata);
        // SmartDashboard.putNumber("arm_right_output", m_motor_right.getAppliedOutput());

        pidController.setP(SmartDashboard.getNumber("Arm kP", 0));
        pidController.setI(SmartDashboard.getNumber("Arm kI", 0));
        pidController.setD(SmartDashboard.getNumber("Arm kD", 0));

        // This method will be called once per scheduler run
        // if (getAngle()>= Arm.Stats.kLimitAngle) 
        // {
        //     setVoltage(0);   
        // }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
