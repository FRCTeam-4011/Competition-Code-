// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private static CANSparkMax rightClimberMotor;
  private static CANSparkMax leftClimberMotor;
  public ClimberSubsystem() {

    //right side climber

   rightClimberMotor = new CANSparkMax(15, MotorType.kBrushless);
   rightClimberMotor.setInverted(true);
   rightClimberMotor.setSmartCurrentLimit(80);
   rightClimberMotor.burnFlash();

    //left side climber

   leftClimberMotor = new CANSparkMax(16, MotorType.kBrushless);
   leftClimberMotor.setInverted(false);
   leftClimberMotor.setSmartCurrentLimit(80);
   leftClimberMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setClimberSpeed(double RightClimberVolts, double LeftClimberVolts) {
    rightClimberMotor.setVoltage(RightClimberVolts);
    leftClimberMotor.setVoltage(LeftClimberVolts);
  }
} 
