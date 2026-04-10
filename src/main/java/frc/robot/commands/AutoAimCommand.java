package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Aims the turret at the AprilTag and sets the hood angle based on distance.
 * Shooter is NOT required — it stays under RT control.
 * Publishes AutoAim/ReadyToShoot when turret is locked and shooter is running.
 */
public class AutoAimCommand extends Command {
    private final TurretSubsystem m_turret;
    private final ShooterAngleSubsystem m_angle;
    private final ShooterSubsystem m_shooter; // read-only, not required

    private static final double DIST_HYSTERESIS_M = 0.15; // min distance change to update hood angle
    private double m_lastDistM = -1.0;
    private double m_lastAngleDeg = Double.NaN;

    public AutoAimCommand(TurretSubsystem turret, ShooterAngleSubsystem angle, ShooterSubsystem shooter) {
        m_turret  = turret;
        m_angle   = angle;
        m_shooter = shooter;
        addRequirements(turret, angle); // shooter NOT a requirement so RT can still control it
    }

    @Override
    public void initialize() {
        m_turret.configureTrackingPipelineAndFilters();
        m_lastDistM   = -1.0;
        m_lastAngleDeg = Double.NaN;
    }

    @Override
    public void execute() {
        m_turret.aimAtAprilTag();

        if (m_turret.hasTarget()) {
            double dist = m_turret.getDistanceToTargetM();
            if (dist > 0.0) {
                // Only recompute the angle when distance changes significantly (reduces noise-driven updates)
                if (m_lastDistM < 0.0 || Math.abs(dist - m_lastDistM) > DIST_HYSTERESIS_M) {
                    m_lastDistM    = dist;
                    m_lastAngleDeg = Constants.ShooterTable.getAngleDeg(m_lastDistM);
                    SmartDashboard.putNumber("AutoAim/DistM",    m_lastDistM);
                    SmartDashboard.putNumber("AutoAim/AngleDeg", m_lastAngleDeg);
                }
            }
        }
        // Always re-send the locked angle every loop — prevents hood from drifting
        // between threshold crossings or when target is briefly lost
        if (!Double.isNaN(m_lastAngleDeg)) {
            m_angle.setAngleDeg(m_lastAngleDeg);
        }

        // On target = limelight TX error is within 2x the deadband
        boolean onTarget = m_turret.hasTarget()
            && Math.abs(m_turret.getTxDeg()) <= Constants.VisionConstants.TX_DEADBAND_DEG * 2.0;
        SmartDashboard.putBoolean("AutoAim/OnTarget",      onTarget);
        SmartDashboard.putBoolean("AutoAim/ReadyToShoot",  onTarget && m_shooter.isRunning());
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
        SmartDashboard.putBoolean("AutoAim/OnTarget",     false);
        SmartDashboard.putBoolean("AutoAim/ReadyToShoot", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
