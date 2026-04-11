// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.util.FuelSim;

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
import frc.robot.subsystems.RollerToShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.FerryAutoAimCommand;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

@SuppressWarnings("unused")
public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop velocity control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController operator2 = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(drivetrain);

    private final ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.CANIds.SHOOTER_MOTOR);
    private final IntakePivotSubsystem m_intake = new IntakePivotSubsystem();
    private final IntakeRollerSubsystem m_intakeRoller = new IntakeRollerSubsystem();
    private final RollerToShooterSubsystem m_rollerToShooter = new RollerToShooterSubsystem();
    private final TurretSubsystem m_turret = new TurretSubsystem();
    private final ShooterAngleSubsystem m_shooterAngle = new ShooterAngleSubsystem();
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();

    private boolean m_intakeDeployed = false;

    // Simulation: track last fuel launch time to simulate ball feed rate
    private double m_lastFuelLaunchTime = 0.0;
    private static final double FUEL_LAUNCH_PERIOD_SEC = 0.5; // Launch one ball every 0.5 seconds

    public RobotContainer() {
        // Register named commands BEFORE building auto chooser
        // Shooter
        NamedCommands.registerCommand("shooterOn", m_shooter.run(() -> {
            double dist = m_turret.getDistanceToTargetM();
            m_shooter.setVoltage(dist > 0.0
                ? Constants.ShooterTable.getVoltage(dist)
                : 10.0);
        }));
        NamedCommands.registerCommand("shooterOff", m_shooter.runOnce(m_shooter::stop));

        NamedCommands.registerCommand("autoAim",
            new AutoAimCommand(m_turret, m_shooterAngle, m_shooter, drivetrain));

        // Intake position
        NamedCommands.registerCommand("intakeZero",
            m_intake.runOnce(m_intake::zero));
        NamedCommands.registerCommand("intakeOut",
            m_intake.runOnce(() -> m_intake.goTo(Constants.IntakeConstants.INTAKE_POSITION)));
        NamedCommands.registerCommand("intakeStow",
            m_intake.runOnce(() -> m_intake.goTo(Constants.IntakeConstants.STOW)));

        // All rollers together
        NamedCommands.registerCommand("rollersOn", Commands.parallel(
            m_intakeRoller.runOnce(m_intakeRoller::start),
            m_indexer.runOnce(m_indexer::start),
            m_rollerToShooter.runOnce(m_rollerToShooter::start)));
        NamedCommands.registerCommand("rollersOff", Commands.parallel(
            m_intakeRoller.runOnce(m_intakeRoller::stop),
            m_indexer.runOnce(m_indexer::stop),
            m_rollerToShooter.runOnce(m_rollerToShooter::stop)));

        // Return turret and hood to zero — runs until at target or 2s timeout
        NamedCommands.registerCommand("turretZero",
            m_turret.run(() -> m_turret.setAngleDeg(0.0))
                    .until(() -> m_turret.atAngle(0.0))
                    .withTimeout(2.0));
        NamedCommands.registerCommand("hoodZero",
            m_shooterAngle.run(() -> m_shooterAngle.setAngleDeg(0.0))
                          .until(() -> m_shooterAngle.atAngle(0.0))
                          .withTimeout(2.0));

        // Individual roller control
        NamedCommands.registerCommand("indexerOn",  m_indexer.runOnce(m_indexer::start));
        NamedCommands.registerCommand("indexerOff", m_indexer.runOnce(m_indexer::stop));
        NamedCommands.registerCommand("rollerToShooterOn",  m_rollerToShooter.runOnce(m_rollerToShooter::start));
        NamedCommands.registerCommand("rollerToShooterOff", m_rollerToShooter.runOnce(m_rollerToShooter::stop));

        // Waits until shooter flywheel is up to speed before feeding balls — 3s safety timeout
        // Uses waitUntil (no subsystem requirement) so it can run alongside shooterOn in a race
        NamedCommands.registerCommand("waitForShooterReady",
            Commands.waitUntil(m_shooter::isAtSpeed).withTimeout(3.0));

        autoChooser = AutoBuilder.buildAutoChooser("Left");
        autoChooser.addOption("None", Commands.none());

        // Simple shooting test for simulation - no vision required
        autoChooser.addOption("Test Shoot (Sim)", Commands.sequence(
            // Set hood to 40 degrees for a nice arc
            m_shooterAngle.runOnce(() -> m_shooterAngle.setAngleDeg(40.0)),
            // Start shooter at 10V
            m_shooter.runOnce(() -> m_shooter.setVoltage(10.0)),
            // Wait 1 second for shooter to spin up
            new WaitCommand(1.0),
            // Start feeding balls
            Commands.parallel(
                m_indexer.runOnce(m_indexer::start),
                m_rollerToShooter.runOnce(m_rollerToShooter::start)
            ),
            // Feed for 10 seconds (20 balls at 0.5s each)
            new WaitCommand(10.0),
            // Stop everything
            Commands.parallel(
                m_shooter.runOnce(m_shooter::stop),
                m_indexer.runOnce(m_indexer::stop),
                m_rollerToShooter.runOnce(m_rollerToShooter::stop)
            ),
            // Reset hood to zero
            m_shooterAngle.runOnce(() -> m_shooterAngle.setAngleDeg(0.0))
        ));

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
        drivetrain.setDefaultCommand(drivetrain.teleopDrive(driver));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // --- Driver ---
        driver.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));   // reset gyro
        driver.b().onTrue(m_turret.runOnce(m_turret::zeroHere));              // zero turret
        driver.x().whileTrue(drivetrain.applyRequest(() -> brake));            // brake
        driver.y().onTrue(m_shooterAngle.runOnce(m_shooterAngle::zeroHere));  // zero hood

        driver.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // --- Operator ---
        // Y: re-zero intake pivot encoder
        operator.y().onTrue(m_intake.runOnce(m_intake::zero));

        // LB: intake pivot in/out toggle
        operator.leftBumper().onTrue(m_intake.runOnce(() -> {
            m_intakeDeployed = !m_intakeDeployed;
            m_intake.goTo(m_intakeDeployed
                ? Constants.IntakeConstants.INTAKE_POSITION
                : Constants.IntakeConstants.STOW);
        }));

        // LT: hold to run intake roller + indexer, release to stop
        // (rollerToShooter excluded so LT + A can run simultaneously)
        operator.leftTrigger(0.1).whileTrue(Commands.parallel(
            m_intakeRoller.startEnd(m_intakeRoller::start, m_intakeRoller::stop),
            m_indexer.startEnd(m_indexer::start, m_indexer::stop)
        ));

        // RB: hold to outtake (intake roller + indexer run in reverse), release to stop
        operator.rightBumper().whileTrue(Commands.parallel(
            m_intakeRoller.startEnd(m_intakeRoller::reverse, m_intakeRoller::stop),
            m_indexer.startEnd(m_indexer::reverse, m_indexer::stop),
            m_rollerToShooter.startEnd(m_rollerToShooter::reverse, m_rollerToShooter::stop)
        ));

        // B alone: ferry auto-aim position — vision tracks tag with offset, NO flywheel
        operator.b().and(operator.a().negate()).whileTrue(
            new FerryAutoAimCommand(m_turret, m_shooterAngle, drivetrain)
        );

        // B + A: ferry shoot with auto-aim — vision tracks + flywheel + feed
        operator.b().and(operator.a()).whileTrue(Commands.parallel(
            new FerryAutoAimCommand(m_turret, m_shooterAngle, drivetrain),
            m_shooter.run(() -> m_shooter.setVoltage(Constants.FerryConstants.VOLTAGE))
                     .finallyDo(m_shooter::stop),
            m_rollerToShooter.startEnd(m_rollerToShooter::start, m_rollerToShooter::stop)
        ));

        // RT alone: auto-aim (turret tracks + hood by distance), NO flywheel
        operator.rightTrigger(0.1).and(operator.a().negate()).whileTrue(
            new AutoAimCommand(m_turret, m_shooterAngle, m_shooter, drivetrain)
        );

        // When RT is released, return turret to zero
        operator.rightTrigger(0.1).onFalse(
            m_turret.run(() -> m_turret.setAngleDeg(0.0))
                    .until(() -> m_turret.atAngle(0.0))
                    .withTimeout(2.0)
        );

        // RT + A: auto-aim shoot — flywheel at table voltage + feed
        operator.rightTrigger(0.1).and(operator.a()).whileTrue(Commands.parallel(
            new AutoAimCommand(m_turret, m_shooterAngle, m_shooter, drivetrain),
            m_shooter.run(() -> {
                double dist = m_turret.getDistanceToTargetM();
                m_shooter.setVoltage(dist > 0.0
                    ? Constants.ShooterTable.getVoltage(dist)
                    : 10.0);
            }).finallyDo(m_shooter::stop),
            m_rollerToShooter.startEnd(m_rollerToShooter::start, m_rollerToShooter::stop),
            m_indexer.startEnd(m_indexer::start, m_indexer::stop)

        ));






        // Turret: right stick X for manual
        m_turret.setDefaultCommand(m_turret.run(() -> {
            double raw = operator.getRightX();
            double deadbanded = Math.abs(raw) > Constants.TurretConstants.MANUAL_DEADBAND ? raw : 0.0;
            m_turret.jogVolts(deadbanded * Constants.TurretConstants.JOG_VOLTAGE);
        }));

        // Hood: when auto-aim is not held, keep the hood at zero.
        m_shooterAngle.setDefaultCommand(m_shooterAngle.run(() -> m_shooterAngle.setAngleDeg(0.0)));
        operator.povDown().onTrue(m_shooterAngle.runOnce(() -> m_shooterAngle.setAngleDeg(Constants.ShooterAngleConstants.MIN_DEG)));
        operator.povLeft().onTrue(m_shooterAngle.runOnce(() -> m_shooterAngle.setAngleDeg(25.0)));
        operator.povUp().onTrue(m_shooterAngle.runOnce(() -> m_shooterAngle.setAngleDeg(40.0)));
        operator.povRight().onTrue(m_shooterAngle.runOnce(() -> m_shooterAngle.setAngleDeg(Constants.ShooterAngleConstants.MAX_DEG)));

        // --- Operator 2 (port 2) ---
        // LT: intake pivot jog toward stow, RT: intake pivot jog toward intake position
        operator2.leftTrigger(0.1).whileTrue(
            m_intake.startEnd(
                () -> m_intake.jogVolts(-Constants.IntakeConstants.JOG_VOLTAGE),
                m_intake::stop
            )
        );
        operator2.rightTrigger(0.1).whileTrue(
            m_intake.startEnd(
                () -> m_intake.jogVolts(Constants.IntakeConstants.JOG_VOLTAGE),
                m_intake::stop
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Updates the fuel simulator when shooting in simulation.
     * Launches fuel when the shooter is running and the feeder is active.
     */
    public void updateShooterSim(FuelSim fuelSim) {
        // Only launch fuel if shooter and feeder are both running
        if (!m_shooter.isRunning() || !m_rollerToShooter.isRunning()) {
            return;
        }

        // Rate limit fuel launches to simulate ball feed rate
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (currentTime - m_lastFuelLaunchTime < FUEL_LAUNCH_PERIOD_SEC) {
            return;
        }
        m_lastFuelLaunchTime = currentTime;

        // Get shooter parameters for launch
        // Estimate velocity from actual shooter motor voltage
        // Typical shooter: 10-12V → ~15-20 m/s exit velocity
        double shooterVoltage = Math.abs(m_shooter.getMotorVoltage());
        double launchVelocity = shooterVoltage * 1.7; // Rough conversion: 10V → 17 m/s, 12V → 20.4 m/s

        // Get hood angle from ShooterAngleSubsystem
        double hoodAngleDeg = m_shooterAngle.getAngleDeg();

        // Get turret angle from TurretSubsystem
        double turretAngleDeg = m_turret.getAngleDeg();

        // Launch the fuel
        fuelSim.launchFuel(
            MetersPerSecond.of(launchVelocity),
            Degrees.of(hoodAngleDeg),
            Degrees.of(turretAngleDeg),
            Meters.of(0.4) // Launch height above ground (shooter exit point)
        );

        // Publish to SmartDashboard for debugging
        SmartDashboard.putNumber("FuelSim/LastLaunchVelocity", launchVelocity);
        SmartDashboard.putNumber("FuelSim/LastLaunchAngle", hoodAngleDeg);
        SmartDashboard.putNumber("FuelSim/LastTurretAngle", turretAngleDeg);
        SmartDashboard.putNumber("FuelSim/LaunchCount", currentTime); // Use time as a changing indicator
    }
}
