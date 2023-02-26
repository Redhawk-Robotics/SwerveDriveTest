// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.modules.CompressorModule;
import frc.robot.test.clawTest;

public class Claw extends CommandBase {
  /** Creates a new Claw. */
  private clawTest claw;
  private CompressorModule compressor;
  
  // FINISH the stuff here
  public Claw(clawTest claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.compressor = CompressorModule.getCompressorModule();
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.compressor.enableAnalog();
    this.claw.stopMotors();
  }

  // ADD TRIGGERS FOR BUTTON
  // WHEN PRESSED OPEN CLAW AND MOVE THE MOTORS

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
