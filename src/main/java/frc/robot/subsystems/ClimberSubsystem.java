// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    // Creates a new ClimberSubsystem.
    private static CANSparkMax rightClimber;
    private static CANSparkMax leftClimber;

    // private static RelativeEncoder rightClimberEncoder;
    // private static RelativeEncoder leftClimberEncoder;

    public ClimberSubsystem() {
        rightClimber = new CANSparkMax(15, MotorType.kBrushless);
        rightClimber.setInverted(false);
        rightClimber.setSmartCurrentLimit(80);
        rightClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setSoftLimit(SoftLimitDirection.kForward, 0);
        rightClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
        rightClimber.enableSoftLimit(SoftLimitDirection.kForward, false);
        rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, false);

        leftClimber = new CANSparkMax(16, MotorType.kBrushless);
        leftClimber.setInverted(false);
        leftClimber.setSmartCurrentLimit(80);
        leftClimber.setIdleMode(IdleMode.kBrake);
        leftClimber.setSoftLimit(SoftLimitDirection.kForward, 0);
        leftClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
        leftClimber.enableSoftLimit(SoftLimitDirection.kForward, false);
        leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, false);

        // rightClimberEncoder = rightClimber.getEncoder();
        // rightClimberEncoder.setPosition(0.0);

        // leftClimberEncoder = leftClimber.getEncoder();
        // leftClimberEncoder.setPosition(0.0);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("left encoder position",
        // leftClimberEncoder.getPosition());
        // SmartDashboard.putNumber("right encoder position",
        // rightClimberEncoder.getPosition());
    }

    public void setClimberVolts(double rightClimberVolts, double leftClimberVolts) {
        rightClimber.setVoltage(rightClimberVolts);
        leftClimber.setVoltage(leftClimberVolts);
    }

}
