// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

	private final SwerveDrive m_swerve;
	private final double m_maxSpeed = 4;

	/** Creates a new SwerveSubsystem. */
	public SwerveSubsystem(File directory) {

		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

		try {
			m_swerve = new SwerveParser(directory).createSwerveDrive(m_maxSpeed);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}

		m_swerve.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot via angle.
    m_swerve.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.

		setupPathPlanner();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		Pose2d whereTheRobotThinksItIsBasedOnTheCamera = LimelightHelpers.getBotPose2d_wpiBlue("");
		if (whereTheRobotThinksItIsBasedOnTheCamera.getX() != 0) {
			m_swerve.addVisionMeasurement(whereTheRobotThinksItIsBasedOnTheCamera, Timer.getFPGATimestamp(), VecBuilder.fill(0.7, 0.7, 0.7));
		}

		// The variable is called "whereTheRobotThinksItIsBasedOnTheCamera" because a non-programmer named the variable.

		// LimelightHelpers.PoseEstimate whereTheRobotThinksItIsBasedOnTheCamera = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
		// System.out.println(whereTheRobotThinksItIsBasedOnTheCamera.tagCount);
		// if (whereTheRobotThinksItIsBasedOnTheCamera.tagCount >= 2) {
		// 	m_swerve.addVisionMeasurement(
		// 		whereTheRobotThinksItIsBasedOnTheCamera.pose,
		// 		whereTheRobotThinksItIsBasedOnTheCamera.timestampSeconds,
		// 		VecBuilder.fill(0.7, 0.7, 9999999)
		// 	);
		// }
	}

	private void setupPathPlanner() {
		AutoBuilder.configureHolonomic(
			m_swerve::getPose, // Robot pose supplier
			m_swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
			m_swerve::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
			m_swerve::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
			new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
				AutoConstants.TRANSLATION_PID, // Translation PID constants
				AutoConstants.ANGLE_PID, // Rotation PID constants
				m_maxSpeed, // Max module speed, in m/s
				m_swerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Drive base radius in meters. Distance from robot center to furthest module.
				new ReplanningConfig() // Default path replanning config. See the API for the options here
			),
			() -> {
				// Boolean supplier that controls when the path will be mirrored for the red alliance
				// This will flip the path being followed to the red side of the field.
				// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
				var alliance = DriverStation.getAlliance();
				return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
			},
			this // Reference to this subsystem to set requirements
		);
	}

	public Command driveToPose(Pose2d pose) {
		// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
			m_swerve.getMaximumVelocity(), 4.0,
			m_swerve.getMaximumAngularVelocity(), Units.degreesToRadians(720));

		// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
			pose,
			constraints,
			0.0, // Goal end velocity in meters/sec
			0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

	public void zeroGyro() {
		m_swerve.zeroGyro();
	}

	public void resetOdometry() {
		m_swerve.resetOdometry(new Pose2d());
	}

	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX) {
		return run(() -> {
			// m_swerve.drive(m_swerve.swerveController.getTargetSpeeds(
			// 	// Math.pow(translationX.getAsDouble(), 3),
			// 	// Math.pow(translationY.getAsDouble(), 3),
			// 	translationX.getAsDouble(),
			// 	translationY.getAsDouble(),
			// 	headingX.getAsDouble(),
			// 	m_swerve.getOdometryHeading().getRadians(),
			// 	m_maxSpeed
			// ));
			m_swerve.drive(
				new ChassisSpeeds(
					translationX.getAsDouble(),
					translationY.getAsDouble(),
					headingX.getAsDouble()
				)
			);
		});
	}
}