// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.EncoderConstants;
import frc.robot.subsystems.ArmSubsystem;


public class MoveArmToSubShotCMD extends Command {
  /** Creates a new MoveArmToSubShotCMD. */
  private final ArmSubsystem armSubsystem;
  private double armVolts;
  

  public MoveArmToSubShotCMD(ArmSubsystem armSubsystem, double armVolts) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.armVolts = armVolts;
    

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Move Arm To Sub Shot Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setArmVoltage(armVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmVoltage(0);
    System.out.println("Move Arm To Sub Shot Stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (armSubsystem.getArmEncoderPOS() <= EncoderConstants.ek_SubShotArmPOS) {
      return true;
    }
    else{return false;}
  }
}
