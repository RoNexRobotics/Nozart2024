package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
	public static final class IntakeConstants {
		public static final int kPivotMotorId = 14;
		public static final int kUpperRollerMotorId = 16;
		public static final int kLowerRollerMotorId = 15;
		public static final int kPivotEncoderChannelA = 0;
		public static final int kPivotEncoderChannelB = 1;
		public static final boolean kPivotEncoderInverted = false;
	}

	public static final class ShooterConstants {
		public static final int kShooterMotor1Id = 13;
		public static final int kShooterMotor2Id = 17;
	}
	
	public static final class ClimberConstants {
		public static final int kLeftArmMotorId = 18;
		public static final int kRightArmMotorId = 19;
		public static final int kLeftLimitSwitchPort = 2;
		public static final int kRightLimitSwitchPort = 3;
	}

	public static final class AutoConstants {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);
		public static final PIDConstants ANGLE_PID = new PIDConstants(5, 0, 0);
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDriverControllerDeadband = 0.06;
		public static final double kOperatorControllerDeadband = 0.1;
	}
}
