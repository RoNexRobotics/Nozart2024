// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_turnEncoder;

  private final PIDController m_turnPIDController = new PIDController(ModuleConstants.kTurnP, ModuleConstants.kTurnI, ModuleConstants.kTurnD);
  private final SlewRateLimiter m_driveLimiter = new SlewRateLimiter(0.8);

  private final String m_moduleName;

  private SwerveModuleState m_lastDesiredState = new SwerveModuleState();

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, double magneticOffset, boolean driveInverted, String moduleName) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = new CANcoder(encoderID);

    // Drive motor configuration
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setInverted(driveInverted);

    // Turn motor configuration
    m_turnMotor.restoreFactoryDefaults();
    m_turnMotor.setInverted(false);

    // Drive encoder configuration
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderPositionFactor);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);

    // Turn encoder configuration
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = magneticOffset;
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    m_turnEncoder.getConfigurator().apply(config);

    // Turn PID controller configuration
    m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turnPIDController.reset();

    m_moduleName = moduleName;
  }

  public void logStuff() {
    // Log stuff about the modules
    if (RobotBase.isReal()) {
      SmartDashboard.putNumber(m_moduleName + " Turn Rotation", getTurnRotation().getDegrees());
      SmartDashboard.putNumber(m_moduleName + " Drive Speed", m_driveMotor.get());
    } else {
      SmartDashboard.putNumber(m_moduleName + " Turn Rotation", m_lastDesiredState.angle.getDegrees());
      SmartDashboard.putNumber(m_moduleName + " Drive Speed", m_lastDesiredState.speedMetersPerSecond);
    }
  }

  public SwerveModuleState getModuleState() {
    // Returns the module state
    return new SwerveModuleState(
      getDriveVelocity(),
      getTurnRotation()
    );
  }

  public SwerveModulePosition getModulePosition() {
    // Returns the module position
    return new SwerveModulePosition(
      getDrivePosition(),
      getTurnRotation()
    );
  }

  public double getDrivePosition() {
    // Returns the drive motor position
    return m_driveEncoder.getPosition();
  }

  public double getDriveVelocity() {
    // Returns the drive motor velocity
    return m_driveEncoder.getVelocity();
  }

  public Rotation2d getTurnRotation() {
    // Returns the turn motor rotation
    return Rotation2d.fromRotations(m_turnEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public void setModuleState(SwerveModuleState desiredState) {

    m_lastDesiredState = desiredState;

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) { // TODO: May have to increase this
      stop();
      return;
    }

    // Optimize the state so the wheel rotates the least distance possible
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getTurnRotation());

    // Set the drive speed
    m_driveMotor.set(m_driveLimiter.calculate(optimizedState.speedMetersPerSecond));

    // Set the turn rotation
    m_turnMotor.set(m_turnPIDController.calculate(getTurnRotation().getRadians(), optimizedState.angle.getRadians()));
  }

  public void stop() {
    // Stop both motors completely
    m_driveMotor.set(0);
    m_turnMotor.set(0);
  }
}
