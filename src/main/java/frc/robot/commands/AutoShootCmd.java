// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCmd extends Command {
  ShooterSubsystem m_shooterSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  Timer timer = new Timer();

  /** Creates a new AutoShootCmd. */
  public AutoShootCmd(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    m_shooterSubsystem.set(1);
    m_intakeSubsystem.setRollerSpeed(1);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 3) {
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.set(0);
    m_intakeSubsystem.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
