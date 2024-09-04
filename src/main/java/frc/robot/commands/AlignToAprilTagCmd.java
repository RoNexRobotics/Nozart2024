// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToAprilTagCmd extends Command {
  // Subsystems
  private SwerveSubsystem m_swerveSubsystem;

  private PIDController m_xPIDController = new PIDController(0, 0, 0);
  private PIDController m_yPIDController = new PIDController(0, 0, 0);
  private PIDController m_rotPIDController = new PIDController(0, 0, 0);

  /** Creates a new AlignToAprilTagCmd. */
  public AlignToAprilTagCmd(SwerveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d botPoseTargetSpace = LimelightHelpers.getBotPose3d_TargetSpace("").toPose2d();
    Pose2d goalPoseTargetSpace = new Pose2d(0, 0, new Rotation2d()); // Goal position in meters relative to the apriltag

    m_swerveSubsystem.drive(
      m_xPIDController.calculate(botPoseTargetSpace.getX(), goalPoseTargetSpace.getX()),
      m_yPIDController.calculate(botPoseTargetSpace.getY(), goalPoseTargetSpace.getY()),
      m_rotPIDController.calculate(botPoseTargetSpace.getRotation().getRadians(), goalPoseTargetSpace.getRotation().getRadians())
    );
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
