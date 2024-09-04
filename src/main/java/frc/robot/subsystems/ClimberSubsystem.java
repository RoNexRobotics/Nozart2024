// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonSRX m_leftClimberMotor;
  private CANSparkMax m_rightClimberMotor;

  private DigitalInput m_leftLimitSwitch = new DigitalInput(ClimberConstants.kLeftLimitSwitchPort);
  private DigitalInput m_rightLimitSwitch = new DigitalInput(ClimberConstants.kRightLimitSwitchPort);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_leftClimberMotor = new TalonSRX(ClimberConstants.kLeftArmMotorId);
    m_rightClimberMotor = new CANSparkMax(ClimberConstants.kRightArmMotorId, MotorType.kBrushed);

    m_leftClimberMotor.configFactoryDefault();
    m_rightClimberMotor.restoreFactoryDefaults();

    m_rightClimberMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LeftLimitSwitch", m_leftLimitSwitch.get());
    SmartDashboard.putBoolean("RightLimitSwitch", m_rightLimitSwitch.get());
  }

  public void set(double speed) {
    setLeftClimberSpeed(speed);
    setRightClimberSpeed(speed);
  }

  public void setLeftClimberSpeed(double speed) {
    // if (speed < 0 && m_leftLimitSwitch.get()) {
      // m_leftClimberMotor.set(ControlMode.PercentOutput, 0);
    // } else {
      m_leftClimberMotor.set(ControlMode.PercentOutput, speed);
    // }
  }

  public void setRightClimberSpeed(double speed) {
    // if (speed < 0 && m_rightLimitSwitch.get()) {
      // m_rightClimberMotor.set(0);
    // } else {
      m_rightClimberMotor.set(speed);
    // }
  }
}
