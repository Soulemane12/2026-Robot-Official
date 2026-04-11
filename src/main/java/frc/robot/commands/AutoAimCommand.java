package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Aims the turret at the AprilTag and sets the hood angle based on distance.
 * When a drivetrain is provided, applies shoot-on-the-move compensation:
 * the turret leads the target based on robot velocity and ball time-of-flight.
 */
public class AutoAimCommand extends Command {
    private final TurretSubsystem m_turret;
    private final ShooterAngleSubsystem m_angle;
    private final ShooterSubsystem m_shooter;       // read-only, not required
    private final CommandSwerveDrivetrain m_drive;  // read-only, not required (may be null)

    private static final double DIST_HYSTERESIS_M = 0.15;
    private double m_lastDistM    = -1.0;
    private double m_lastAngleDeg = Double.NaN;

    public AutoAimCommand(TurretSubsystem turret, ShooterAngleSubsystem angle,
                          ShooterSubsystem shooter) {
        this(turret, angle, shooter, null);
    }

    public AutoAimCommand(TurretSubsystem turret, ShooterAngleSubsystem angle,
                          ShooterSubsystem shooter, CommandSwerveDrivetrain drive) {
        m_turret  = turret;
        m_angle   = angle;
        m_shooter = shooter;
        m_drive   = drive;
        addRequirements(turret, angle); // drivetrain/shooter NOT requirements
    }

    @Override
    public void initialize() {
        m_turret.configureTrackingPipelineAndFilters();
        m_lastDistM    = -1.0;
        m_lastAngleDeg = Double.NaN;
    }

    @Override
    public void execute() {
        double leadDeg      = 0.0;
        double effectiveDist = -1.0;

        if (m_turret.hasTarget()) {
            double dist = m_turret.getDistanceToTargetM();
            if (dist > 0.0) {
                // --- Shoot-on-the-move velocity compensation ---
                if (m_drive != null) {
                    var state   = m_drive.getState();
                    double heading = state.Pose.getRotation().getRadians();
                    double vx   = state.Speeds.vxMetersPerSecond;
                    double vy   = state.Speeds.vyMetersPerSecond;

                    // Convert robot-relative velocity to field frame
                    double vx_f = vx * Math.cos(heading) - vy * Math.sin(heading);
                    double vy_f = vx * Math.sin(heading) + vy * Math.cos(heading);

                    // Unit vector along turret line-of-sight in field frame
                    double turretFieldRad = heading + Math.toRadians(m_turret.getAngleDeg());
                    double tdx = Math.cos(turretFieldRad);
                    double tdy = Math.sin(turretFieldRad);

                    // Lateral velocity (perpendicular to line-of-sight)
                    double v_lateral = -vx_f * tdy + vy_f * tdx;

                    double tof         = Constants.ShooterTable.getShotTime(dist);
                    double lateralDisp = v_lateral * tof;
                    leadDeg      = Math.toDegrees(Math.atan2(lateralDisp, dist));
                    effectiveDist = Math.sqrt(dist * dist + lateralDisp * lateralDisp);

                    SmartDashboard.putNumber("AutoAim/LeadDeg",      leadDeg);
                    SmartDashboard.putNumber("AutoAim/EffectiveDist", effectiveDist);
                } else {
                    effectiveDist = dist;
                }

                // Only recompute hood angle when distance changes significantly
                double lookupDist = effectiveDist > 0.0 ? effectiveDist : dist;
                if (m_lastDistM < 0.0 || Math.abs(dist - m_lastDistM) > DIST_HYSTERESIS_M) {
                    m_lastDistM    = dist;
                    m_lastAngleDeg = Constants.ShooterTable.getAngleDeg(lookupDist);
                    SmartDashboard.putNumber("AutoAim/DistM",    m_lastDistM);
                    SmartDashboard.putNumber("AutoAim/AngleDeg", m_lastAngleDeg);
                }
            }
        }

        // Aim turret with lead angle (0 if stationary or no drivetrain)
        m_turret.aimAtAprilTag(false, leadDeg);

        // Always re-send the locked angle — prevents hood from drifting
        if (!Double.isNaN(m_lastAngleDeg)) {
            m_angle.setAngleDeg(m_lastAngleDeg);
        }

        boolean onTarget = m_turret.hasTarget()
            && Math.abs(m_turret.getTxDeg()) <= Constants.VisionConstants.TX_DEADBAND_DEG * 2.0;
        SmartDashboard.putBoolean("AutoAim/OnTarget",     onTarget);
        SmartDashboard.putBoolean("AutoAim/ReadyToShoot", onTarget && m_shooter.isRunning());
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
