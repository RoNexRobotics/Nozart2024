package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
	public static final class IntakeConstants {
		public static final int kIntakePivotMotorId = 14;
		public static final int kIntakeMotorId = 15;
		public static final int kIntakePivotEncoderChannelA = 0;
		public static final int kIntakePivotEncoderChannelB = 1;
	}

	public static final class ShooterConstants {
		public static final int kShooterMotorId = 13;
	}

	public static final class AutoConstants {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);
		public static final PIDConstants ANGLE_PID = new PIDConstants(5, 0, 0);
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
	}
}
