// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // Subsystems
  private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Controllers
  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {

    // Setup default commands
    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveCommand(
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX(),
      () -> m_driverController.getRightX()
    ));

    registerNamedCommands();
    setupAutoChooser();
    configureBindings();
  }

  private void registerNamedCommands() {}

  private void setupAutoChooser() {

    // List of autos
    m_autoChooser.addOption("None", Commands.none());
    m_autoChooser.addOption("Path 1", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Path 1")));
    m_autoChooser.addOption("Auto 1", AutoBuilder.buildAuto("Auto 1"));
    m_autoChooser.addOption("Auto 2", AutoBuilder.buildAuto("Auto 2"));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void configureBindings() {
    m_driverController.y().onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro, m_swerveSubsystem));

    m_driverController.b().onTrue(new InstantCommand(m_swerveSubsystem::resetOdometry, m_swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
