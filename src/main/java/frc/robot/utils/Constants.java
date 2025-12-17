package frc.robot.utils;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;


public class Constants {
    public class DriveConstants {
        public static final double MAX_SPEED = 
            TunerConstants.kSpeedAt12Volts
            .in(MetersPerSecond); 
        public static final double MAX_ACCELERATION = 
            MetersPerSecondPerSecond
            .of(10)
            .in(MetersPerSecondPerSecond);
        public static final double MAX_ANGULAR_SPEED = 
            RotationsPerSecond
            .of(2)
            .in(RadiansPerSecond);
        public static final double MAX_ANGULAR_ACCELERATION = 
            RotationsPerSecond
            .of(3)
            .in(RadiansPerSecond); 

        public static final PathConstraints DEFAULT_CONSTRAINTS = 
            new PathConstraints(
                MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION
            );
        
    }
}
