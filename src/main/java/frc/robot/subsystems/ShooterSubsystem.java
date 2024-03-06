// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private VictorSPX m_motor = new VictorSPX(ShooterConstants.kShooterMotorId);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_motor.configFactoryDefault();

    VictorSPXConfiguration config = new VictorSPXConfiguration();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double speed) {
    m_motor.set(VictorSPXControlMode.PercentOutput, speed);
  }
}
