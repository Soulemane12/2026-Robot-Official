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
import frc.robot.subsystems.BallCounterSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.TrackHubCommand;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

@SuppressWarnings("unused")
public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;
    // Vision subsystem with callback to update drivetrain odometry
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(drivetrain::addVisionMeasurement);
    private final BallCounterSubsystem m_ballCounter = new BallCounterSubsystem();

    // Store previous Limelight settings for restoration after vision alignment
    private int prevPipeline = 0;
    private int prevLedMode = 0;

    public RobotContainer() {
        // Register named commands BEFORE building auto chooser
        NamedCommands.registerCommand("p",
            new TrackHubCommand(drivetrain, m_visionSubsystem));

        autoChooser = AutoBuilder.buildAutoChooser("circle");

        configureBindings();
        FollowPathCommand.warmupCommand().schedule();

        SmartDashboard.putData("Auto Mode", autoChooser);

        // Continuous monitoring of trigger for debugging - schedule it to run always
        new Command() {
            @Override
            public void execute() {
                double triggerValue = joystick.getLeftTriggerAxis();
                SmartDashboard.putNumber("Debug/LeftTriggerRaw", triggerValue);
                SmartDashboard.putBoolean("Debug/TriggerAboveThreshold", triggerValue > 0.1);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        }.ignoringDisable(true).schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Default teleop drive command using the new teleopDrive method
        drivetrain.setDefaultCommand(drivetrain.teleopDrive(joystick));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Vision alignment - press LEFT TRIGGER (LT, not LB!) to toggle auto-rotate toward AprilTag
        joystick.leftTrigger(0.1).toggleOnTrue(
            drivetrain.visionAlignDrive(joystick, m_visionSubsystem)
                .beforeStarting(() -> {
                    drivetrain.resetAimLimiter();

                    // Store current settings before switching
                    prevPipeline = m_visionSubsystem.getPipeline();
                    prevLedMode = m_visionSubsystem.getLEDMode();

                    // Switch to AprilTag pipeline and turn on LEDs
                    m_visionSubsystem.setPipeline(0);   // Set to your AprilTag pipeline index
                    m_visionSubsystem.setLEDMode(3);    // Turn on LEDs while aiming
                })
                .finallyDo(interrupted -> {
                    drivetrain.resetAimLimiter();

                    // Restore previous settings
                    m_visionSubsystem.setPipeline(prevPipeline);
                    m_visionSubsystem.setLEDMode(prevLedMode);
                })
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Ball counter - press BACK button to reset count
        // DISABLED - Uncomment when CANrange is working
        joystick.rightTrigger().onTrue(m_ballCounter.runOnce(m_ballCounter::resetCount));


   /* 
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

*/
        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
