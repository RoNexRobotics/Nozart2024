// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private VictorSPX m_pivotMotor = new VictorSPX(IntakeConstants.kIntakePivotMotorId);
  private VictorSPX m_motor = new VictorSPX(IntakeConstants.kIntakeMotorId);

  private Encoder m_pivotEncoder = new Encoder(IntakeConstants.kIntakePivotEncoderChannelA, IntakeConstants.kIntakePivotEncoderChannelB);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
