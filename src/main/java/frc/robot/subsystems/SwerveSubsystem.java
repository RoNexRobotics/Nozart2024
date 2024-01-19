// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule m_frontLeftModule = new SwerveModule(
    DriveConstants.kFrontLeftDriveID, DriveConstants.kFrontLeftTurnID, DriveConstants.kFrontLeftEncoderID, DriveConstants.kFrontLeftMagneticOffset, DriveConstants.kFrontLeftDriveInverted, "FL");
  private final SwerveModule m_frontRightModule = new SwerveModule(
    DriveConstants.kFrontRightDriveID, DriveConstants.kFrontRightTurnID, DriveConstants.kFrontRightEncoderID, DriveConstants.kFrontRightMagneticOffset, DriveConstants.kFrontRightDriveInverted, "FR");
  private final SwerveModule m_rearLeftModule = new SwerveModule(
    DriveConstants.kRearLeftDriveID, DriveConstants.kRearLeftTurnID, DriveConstants.kRearLeftEncoderID, DriveConstants.kRearLeftMagneticOffset, DriveConstants.kRearLeftDriveInverted, "RL");
  private final SwerveModule m_rearRightModule = new SwerveModule(
    DriveConstants.kRearRightDriveID, DriveConstants.kRearRightTurnID, DriveConstants.kRearRightEncoderID, DriveConstants.kRearRightMagneticOffset, DriveConstants.kRearRightDriveInverted, "RR");
  
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    getHeading(),
    new SwerveModulePosition[] {
      m_frontLeftModule.getModulePosition(),
      m_frontRightModule.getModulePosition(),
      m_rearLeftModule.getModulePosition(),
      m_rearRightModule.getModulePosition(),
    }
  );

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();

  private Field2d m_field = new Field2d();
  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier (Robot relative)
      this::drive, // Method to drive the robot (Robot relative)
      AutoConstants.kHolonomicPathFollowerConfig,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
  }

  @Override
  public void periodic() {
    // Update odometry
    m_odometry.update(
      getHeading(),
      new SwerveModulePosition[] {
        m_frontLeftModule.getModulePosition(),
        m_frontRightModule.getModulePosition(),
        m_rearLeftModule.getModulePosition(),
        m_rearRightModule.getModulePosition(),
      }
    );

    m_frontLeftModule.logStuff();
    m_frontRightModule.logStuff();
    m_rearLeftModule.logStuff();
    m_rearRightModule.logStuff();

    // Update field
    m_field.setRobotPose(getPose());
    SmartDashboard.putData(m_field);

    // AdvantageKit logging
    Logger.recordOutput("Robot Angle", getHeading());
    Logger.recordOutput("Robot Pose", getPose());
  }

  public Rotation2d getHeading() {
    // Returns the gyro heading (inverted because we want positive clockwise rotation)
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void resetHeading() {
    // Resets the gyro heading
    m_gyro.reset();
  }

  public Pose2d getPose() {
    // Returns the odometry pose
    return new Pose2d(
      new Translation2d(m_odometry.getPoseMeters().getX(), -m_odometry.getPoseMeters().getY()),
      m_odometry.getPoseMeters().getRotation()
    );
  }

  public void resetPose() {
    // Resets the odometry pose
    resetPose(new Pose2d());
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
      getHeading(),
      new SwerveModulePosition[] {
        m_frontLeftModule.getModulePosition(),
        m_frontRightModule.getModulePosition(),
        m_rearLeftModule.getModulePosition(),
        m_rearRightModule.getModulePosition(),
      },
      pose
    );
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return m_chassisSpeeds;
  }

  private void drive(ChassisSpeeds chassisSpeeds) {
    drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false);
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed) {
    drive(xSpeed, ySpeed, rotSpeed, false);
  }

  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    // Convert controller inputs to chassis speeds
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    m_chassisSpeeds = speeds;

    if (fieldRelative) {
      // If field-relative mode, convert to field-relative chassis speeds
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, Rotation2d.fromRadians(-getHeading().getRadians()));
    }

    // Convert chassis speeds to swerve module states
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Set the module states
    setModuleStates(states);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    // Reduce the wheel speeds to attainable values
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);

    // Set each module state
    m_frontLeftModule.setModuleState(states[0]);
    m_frontRightModule.setModuleState(states[1]);
    m_rearLeftModule.setModuleState(states[2]);
    m_rearRightModule.setModuleState(states[3]);
  }

  public void stopModules() {
    // Stop each module completely
    m_frontLeftModule.stop();
    m_frontRightModule.stop();
    m_rearLeftModule.stop();
    m_rearRightModule.stop();
  }
}
