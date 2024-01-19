// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTeleopCmd extends Command {
  private final SwerveSubsystem m_swerveSubsystem;

  private final CommandXboxController m_controller;

  /** Creates a new DriveTeleopCmd. */
  public DriveTeleopCmd(SwerveSubsystem swerveSubsystem, CommandXboxController controller) {
    m_swerveSubsystem = swerveSubsystem;
    m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.kDriverControllerDeadband) * 0.4;
    double ySpeed = MathUtil.applyDeadband(m_controller.getLeftX(), OperatorConstants.kDriverControllerDeadband) * 0.4;
    double rotSpeed = MathUtil.applyDeadband(m_controller.getRightX(), OperatorConstants.kDriverControllerDeadband) * 0.4;

    if (m_controller.rightBumper().getAsBoolean()) {
      m_swerveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);
    } else {
      m_swerveSubsystem.drive(xSpeed, ySpeed, rotSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
