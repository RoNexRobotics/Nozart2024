package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
	public static final class AutoConstants {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(20, 0, 0);
		public static final PIDConstants ANGLE_PID = new PIDConstants(1, 0, 0);
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
	}
}
