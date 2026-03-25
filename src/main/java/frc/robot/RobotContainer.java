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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.RollerToShooterSubsystem;
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

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;
    // Vision subsystem with callback to update drivetrain odometry
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(drivetrain::addVisionMeasurement);

    private final ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.CANIds.SHOOTER_MOTOR);
    private final IntakePivotSubsystem m_intake = new IntakePivotSubsystem();
    private final IntakeRollerSubsystem m_intakeRoller = new IntakeRollerSubsystem();
    private final TurretSubsystem m_turret = new TurretSubsystem();
    private final ShooterAngleSubsystem m_shooterAngle = new ShooterAngleSubsystem();
    private final RollerToShooterSubsystem m_rollerToShooter = new RollerToShooterSubsystem();

    // Store previous Limelight settings for restoration after vision alignment
    private int prevPipeline = 0;
    private int prevLedMode = 0;

    public RobotContainer() {
        // Register named commands BEFORE building auto chooser
        NamedCommands.registerCommand("p",
            new TrackHubCommand(drivetrain, m_visionSubsystem));
        NamedCommands.registerCommand("shooterOn",
            m_shooter.runOnce(m_shooter::start));
        NamedCommands.registerCommand("shooterOff",
            m_shooter.runOnce(m_shooter::stop));
        NamedCommands.registerCommand("rollerOn",
            m_rollerToShooter.runOnce(m_rollerToShooter::start));
        NamedCommands.registerCommand("rollerOff",
            m_rollerToShooter.runOnce(m_rollerToShooter::stop));

        autoChooser = AutoBuilder.buildAutoChooser("");
        autoChooser.setDefaultOption("None", Commands.none());
        autoChooser.addOption("Shoot 3 Balls", Commands.sequence(
            m_shooter.runOnce(m_shooter::start),
            new WaitCommand(1.5),
            m_rollerToShooter.runOnce(m_rollerToShooter::start),
            new WaitCommand(0.5),
            m_rollerToShooter.runOnce(m_rollerToShooter::stop),
            new WaitCommand(0.3),
            m_rollerToShooter.runOnce(m_rollerToShooter::start),
            new WaitCommand(0.5),
            m_rollerToShooter.runOnce(m_rollerToShooter::stop),
            new WaitCommand(0.3),
            m_rollerToShooter.runOnce(m_rollerToShooter::start),
            new WaitCommand(0.5),
            m_rollerToShooter.runOnce(m_rollerToShooter::stop),
            m_shooter.runOnce(m_shooter::stop)
        ));

        configureBindings();
        FollowPathCommand.warmupCommand().schedule();

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Default teleop drive command using the new teleopDrive method
        drivetrain.setDefaultCommand(drivetrain.teleopDrive(driver));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Vision alignment - press LEFT TRIGGER (LT, not LB!) to toggle auto-rotate toward AprilTag
        driver.leftTrigger(0.1).toggleOnTrue(
            drivetrain.visionAlignDrive(driver, m_visionSubsystem)
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

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        driver.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        driver.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Operator controls - Shooter toggle on A button
        operator.a().onTrue(m_shooter.runOnce(m_shooter::toggle));

        // Intake controls - Calibration and position control
        driver.a().onTrue(m_intake.runOnce(m_intake::zero));

        operator.rightBumper().whileTrue(
            m_intake.startEnd(
                () -> m_intake.jogVolts(-Constants.IntakeConstants.JOG_VOLTAGE),
                m_intake::stop
            )
        );

        operator.leftBumper().whileTrue(
            m_intake.startEnd(
                () -> m_intake.jogVolts(Constants.IntakeConstants.JOG_VOLTAGE),
                m_intake::stop
            )
        );

        operator.x().onTrue(m_intakeRoller.runOnce(m_intakeRoller::toggle));
        operator.b().onTrue(m_rollerToShooter.runOnce(m_rollerToShooter::toggle));

        operator.rightTrigger().onTrue(m_intake.runOnce(() -> m_intake.goTo(Constants.IntakeConstants.STOW)));
        operator.leftTrigger().onTrue(m_intake.runOnce(() -> m_intake.goTo(Constants.IntakeConstants.EXTENDED)));
        operator.y().onTrue(m_intake.runOnce(() -> m_intake.goTo(Constants.IntakeConstants.INTAKE_POSITION)));

        // Toggle intake pivot limits on/off
        operator.povUp().onTrue(m_intake.runOnce(m_intake::toggleLimits));

        // Shooter angle — left stick Y for manual jog (up = raise, negate because stick up = negative Y)
        m_shooterAngle.setDefaultCommand(m_shooterAngle.run(
            () -> m_shooterAngle.jogVolts(-operator.getLeftY() * Constants.ShooterAngleConstants.JOG_VOLTAGE)
        ));

        // Turret — right stick X for manual jog, D-pad presets
        m_turret.setDefaultCommand(m_turret.run(
            () -> m_turret.jogVolts(operator.getRightX() * Constants.TurretConstants.JOG_VOLTAGE)
        ));
        operator.povLeft().onTrue(m_turret.runOnce(() -> m_turret.setAngleDeg(-90)));
        operator.povRight().onTrue(m_turret.runOnce(() -> m_turret.setAngleDeg(90)));
        operator.povDown().onTrue(m_turret.runOnce(() -> m_turret.setAngleDeg(0)));

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
