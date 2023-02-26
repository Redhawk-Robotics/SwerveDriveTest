// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.test.armTest;

public class armHigh extends CommandBase {
  /** Creates a new armHigh. */
  private armTest arm;
  private PIDController armPID;

  public armHigh(armTest arm, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.armPID = new PIDController(0,0,0);
    armPID.setSetpoint(setPoint);

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArmPIDCmd started!");
    armPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  double speed = armPID.calculate(arm.getEncoderMetersLeft(0));
  arm.setMotor(speed);
  //TODO either use two pids with 
  //a difference of the error that outputs to the motor or only use one, this is for tmmrw
  //TODO subsystem for pneumatics

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ArmPIDCommand is completed:)))");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
