// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private VictorSPX m_pivotMotor;
  private CANSparkMax m_upperRollerMotor;
  private CANSparkMax m_lowerRollerMotor;

  private Encoder m_pivotEncoder;

  private PIDController m_pidController = new PIDController(0, 0, 0);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_pivotMotor = new VictorSPX(IntakeConstants.kPivotMotorId);
    m_upperRollerMotor = new CANSparkMax(IntakeConstants.kUpperRollerMotorId, MotorType.kBrushed);
    m_lowerRollerMotor = new CANSparkMax(IntakeConstants.kLowerRollerMotorId, MotorType.kBrushed);

    m_pivotMotor.configFactoryDefault();
    m_upperRollerMotor.restoreFactoryDefaults();
    m_lowerRollerMotor.restoreFactoryDefaults();

    m_pivotEncoder = new Encoder(
      IntakeConstants.kPivotEncoderChannelA,
      IntakeConstants.kPivotEncoderChannelB,
      IntakeConstants.kPivotEncoderInverted,
      EncodingType.k1X);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Intake Angle", m_pivotEncoder.get());
  }

  public void setAngleSetpoint(double setpoint) {
    m_pivotMotor.set(ControlMode.PercentOutput, m_pidController.calculate(m_pivotEncoder.get(), setpoint));
  }

  public void setRollerSpeed(double speed) {
    m_upperRollerMotor.set(speed);
    m_lowerRollerMotor.set(speed);
  }
}
