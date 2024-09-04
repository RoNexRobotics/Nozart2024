// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PivotIntakeCmd extends Command {
  private IntakeSubsystem m_intakeSubsystem;

  private DoubleSupplier m_speedSupplier;

  private Joystick m_joystick;

  /** Creates a new PivotIntakeCmd. */
  public PivotIntakeCmd(IntakeSubsystem intakeSubsystem, Joystick joystick) {
    m_intakeSubsystem = intakeSubsystem;
    // m_speedSupplier = speedSupplier;

    m_joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setAngleSpeed(-m_joystick.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setAngleSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
