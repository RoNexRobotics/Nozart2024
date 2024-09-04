// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private VictorSPX m_motor1;
  private VictorSPX m_motor2;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_motor1 = new VictorSPX(ShooterConstants.kShooterMotor1Id);
    m_motor2 = new VictorSPX(ShooterConstants.kShooterMotor2Id);
    m_motor1.configFactoryDefault();
    m_motor2.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Shooter Motor 1 Speed", m_motor1.getMotorOutputPercent());
    Logger.recordOutput("Shooter Motor 2 Speed", m_motor2.getMotorOutputPercent());
  }

  public void set(double speed) {
    m_motor1.set(VictorSPXControlMode.PercentOutput, speed);
    m_motor2.set(VictorSPXControlMode.PercentOutput, speed);
  }
}
