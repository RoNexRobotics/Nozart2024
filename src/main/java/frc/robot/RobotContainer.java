// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTagCmd;
import frc.robot.commands.AutoShootCmd;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.MoveLeftArmCmd;
import frc.robot.commands.MoveRightArmCmd;
import frc.robot.commands.PivotIntakeCmd;
import frc.robot.commands.RunIntakeCmd;
import frc.robot.commands.RunShooterCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // Subsystems
  private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  // Controllers
  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  // Commands
  private AlignToAprilTagCmd m_alignToAprilTagCmd = new AlignToAprilTagCmd(m_swerveSubsystem);
  private ClimberCmd m_climberCmd = new ClimberCmd(m_climberSubsystem, m_operatorController);
  private AutoShootCmd m_autoShootCmd = new AutoShootCmd(m_shooterSubsystem, m_intakeSubsystem);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {

    // Setup default commands
    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriverControllerDeadband) * 3,
      () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriverControllerDeadband) * 3,
      () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriverControllerDeadband) * 3
    ));

    m_intakeSubsystem.setDefaultCommand(new PivotIntakeCmd(m_intakeSubsystem, m_operatorController));

    m_climberSubsystem.setDefaultCommand(m_climberCmd);

    registerNamedCommands();
    setupAutoChooser();
    configureBindings();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("AutoShoot", m_autoShootCmd);
  }

  private void setupAutoChooser() {

    // List of autos
    m_autoChooser.addOption("None", Commands.none());
    m_autoChooser.addOption("PreShot1NoteLeave", AutoBuilder.buildAuto("PreShot1NoteLeave"));
    m_autoChooser.addOption("RightSpkrLeave", AutoBuilder.buildAuto("RightSpkrLeave"));
    m_autoChooser.addOption("New Auto", AutoBuilder.buildAuto("New Auto"));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void configureBindings() {
    
    new JoystickButton(m_operatorController, 1).whileTrue(new RunShooterCmd(m_shooterSubsystem, m_intakeSubsystem, 1));

    new JoystickButton(m_operatorController, 2).whileTrue(new RunShooterCmd(m_shooterSubsystem, m_intakeSubsystem, 0.4));
    new JoystickButton(m_operatorController, 7).whileTrue(new RunIntakeCmd(m_intakeSubsystem, -1));

    m_driverController.x().whileTrue(new MoveRightArmCmd(m_climberSubsystem, 1));
    m_driverController.y().whileTrue(new MoveLeftArmCmd(m_climberSubsystem, 1));
    m_driverController.a().whileTrue(new MoveRightArmCmd(m_climberSubsystem, -1));
    m_driverController.b().whileTrue(new MoveLeftArmCmd(m_climberSubsystem, -1));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
    // return null;
  }
}
