package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class TrackHubCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_vision;

    // Control parameters
    private static final double TX_KP = 0.02;
    private static final double LOST_TARGET_TIMEOUT = 0.5;

    // State tracking
    private double m_lastRotation = 0.0;
    private double m_lastSeenTime = 0.0;

    public TrackHubCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_lastRotation = 0.0;
        m_lastSeenTime = Timer.getFPGATimestamp();

        // Optionally switch to AprilTag pipeline and enable LEDs
        m_vision.setPipeline(0);
        m_vision.setLEDMode(3); // Force LEDs on
    }

    @Override
    public void execute() {
        double rotation = 0.0;

        if (m_vision.hasValidTarget()) {
            // Get horizontal offset (TX)
            double tx = m_vision.getTargetTX();

            // Calculate rotation to face target (negative because positive TX means target is to the right)
            rotation = -TX_KP * tx;

            // Update last seen time and rotation
            m_lastSeenTime = Timer.getFPGATimestamp();
            m_lastRotation = rotation;
        } else {
            // Target lost - use last rotation for timeout period
            double timeSinceLastSeen = Timer.getFPGATimestamp() - m_lastSeenTime;
            if (timeSinceLastSeen < LOST_TARGET_TIMEOUT) {
                rotation = m_lastRotation;
            } else {
                rotation = 0.0;
            }
        }

        // Apply rotation only (no translation - PathPlanner handles that)
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotation);
        m_drivetrain.setControl(
            new com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(speeds)
        );

        // Debug output
        SmartDashboard.putNumber("TrackHub/TX", m_vision.getTargetTX());
        SmartDashboard.putNumber("TrackHub/Rotation", rotation);
        SmartDashboard.putBoolean("TrackHub/HasTarget", m_vision.hasValidTarget());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop rotation
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
        m_drivetrain.setControl(
            new com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(speeds)
        );
    }

    @Override
    public boolean isFinished() {
        // Command runs until manually stopped or interrupted
        return false;
    }
}
