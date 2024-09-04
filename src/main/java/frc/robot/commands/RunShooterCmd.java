// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCmd extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;

  private Timer m_timer = new Timer();

  private double m_speed;

  /** Creates a new RunShooterCmd. */
  public RunShooterCmd(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double speed) {
    m_shooterSubsystem = shooterSubsystem;
    m_intakeSubsystem = intakeSubsystem;

    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() < 1.2) {
      m_shooterSubsystem.set(m_speed);
    } else if (m_timer.get() >= 1.2) {
      m_intakeSubsystem.setRollerSpeed(1);
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
