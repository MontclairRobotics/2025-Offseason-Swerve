package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;
import frc.robot.generated.TunerConstants;

public class Constants {
    public class DriveConstants {
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ACCELERATION =
                MetersPerSecondPerSecond.of(10).in(MetersPerSecondPerSecond);
        public static final double MAX_ANGULAR_SPEED = RotationsPerSecond.of(2).in(RadiansPerSecond);
        public static final double MAX_ANGULAR_ACCELERATION =
                RotationsPerSecond.of(3).in(RadiansPerSecond);

        public static final PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);
    }
}
