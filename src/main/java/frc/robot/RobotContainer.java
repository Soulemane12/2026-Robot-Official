// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.FollowAprilTagCommand;

//import com.pathplanner.lib.auto.AutoBuilder;

@SuppressWarnings("unused")
public class RobotContainer {
   // private final SendableChooser<Command> autoChooser;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

    // Commands
    // AprilTag follow command - robot maintains distance and faces the tag
    // Parameters: (VisionSubsystem, Drivetrain, targetTY, targetAngle)
    // - targetTY: The TY value to maintain (controls distance). Higher = closer. Start with 0.
    // - targetAngle: Usually 0 to face the tag head-on
    private final FollowAprilTagCommand followAprilTagCommand =
        new FollowAprilTagCommand(m_visionSubsystem, drivetrain, 0.0, 0.0);

    public RobotContainer() {
     //   autoChooser = AutoBuilder.buildAutoChooser("New Auto");

        configureBindings();
      //  SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // A button = Brake
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // B button = Point wheels
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // X button = Reset field-centric heading
        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // ===== APRILTAG FOLLOWING =====
        // Y button = Follow AprilTag (HOLD to follow)
        // Hold up a board with an AprilTag and move it around!
        // The robot will face it and maintain distance
        joystick.y().whileTrue(followAprilTagCommand);

        // Right bumper = Follow AprilTag (alternative binding)
        joystick.rightBumper().whileTrue(followAprilTagCommand);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

  //  public Command getAutonomousCommand() {
      //  return autoChooser.getSelected();
  //  }
}
