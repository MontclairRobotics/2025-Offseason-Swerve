// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.Constants.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.PoseUtils;

public class RobotContainer {

    private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED);

    // Joysticks
    public static final CommandPS5Controller driverController = new CommandPS5Controller(0);

    // Subsystems
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.driveJoystickInputCommand());

        // Robot relative
        driverController.L2().onTrue(drivetrain.toRobotRelativeCommand()).onFalse(drivetrain.toFieldRelativeCommand());

        // 90 degree buttons
        driverController
                .triangle()
                .onTrue(drivetrain.alignToAngleFieldRelativeCommand(
                        PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(0)), false));
        driverController
                .square()
                .onTrue(drivetrain.alignToAngleFieldRelativeCommand((Rotation2d.fromDegrees(-54)), false));
        driverController
                .cross()
                .onTrue(drivetrain.alignToAngleFieldRelativeCommand(
                        PoseUtils.flipRotAlliance(Rotation2d.fromDegrees(180)), false));
        driverController
                .circle()
                .onTrue(drivetrain.alignToAngleFieldRelativeCommand(Rotation2d.fromDegrees(54), false));

        // zeros gyro
        driverController.touchpad().onTrue(drivetrain.zeroGyroCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
