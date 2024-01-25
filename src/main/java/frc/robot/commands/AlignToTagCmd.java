// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToTagCmd extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private NetworkTableInstance m_inst;
  private NetworkTable m_table;

  private PIDController m_xPIDController = new PIDController(0.05, 0, 0);
  private PIDController m_yPIDController = new PIDController(0.01, 0, 0);
  private PIDController m_rotPIDController = new PIDController(0.01, 0, 0);

  /** Creates a new AlignToTagCmd. */
  public AlignToTagCmd(SwerveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;

    m_inst = NetworkTableInstance.getDefault();
    m_table = m_inst.getTable("limelight");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double tx = m_table.getEntry("tx").getDouble(0);
    // double ta = m_table.getEntry("ta").getDouble(0);
    // double[] botpose = m_table.getEntry("botpose").getDoubleArray(new double[7]);
    double tx = LimelightHelpers.getTX("");
    double ta = LimelightHelpers.getTA("");
    double[] botpose = LimelightHelpers.getBotPose("");

    double xSpeed = 0;
    double ySpeed = 0;

    double rangeGoal = 6;

    if (Math.abs(tx) <= 2) {
      ySpeed = 0;
    } else {
      ySpeed = -m_yPIDController.calculate(tx, 0);
    }

    if (ta == 0) {
      ta = rangeGoal;
    }

    if (Math.abs(ta - rangeGoal) <= 1) {
      xSpeed = 0;
    } else {
      xSpeed = m_xPIDController.calculate(ta, rangeGoal);
    }

    if (ta < 2) {
      m_swerveSubsystem.drive(
        xSpeed,
        0,
        -m_rotPIDController.calculate(botpose[5], 0)
      );
    } else {
      m_swerveSubsystem.drive(
        xSpeed,
        ySpeed,
        -m_rotPIDController.calculate(botpose[5], 0)
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
