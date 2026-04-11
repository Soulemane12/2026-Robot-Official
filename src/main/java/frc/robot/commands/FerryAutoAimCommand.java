package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Ferry mode auto-aim command.
 * Uses vision tracking with alliance-specific tag filtering and tag-specific turret offsets.
 * Sets hood to fixed ferry angle. When a drivetrain is provided, applies shoot-on-the-move
 * lead compensation to the turret.
 */
public class FerryAutoAimCommand extends Command {
    private final TurretSubsystem m_turret;
    private final ShooterAngleSubsystem m_angle;
    private final CommandSwerveDrivetrain m_drive; // read-only, not required (may be null)

    public FerryAutoAimCommand(TurretSubsystem turret, ShooterAngleSubsystem angle) {
        this(turret, angle, null);
    }

    public FerryAutoAimCommand(TurretSubsystem turret, ShooterAngleSubsystem angle,
                               CommandSwerveDrivetrain drive) {
        m_turret = turret;
        m_angle  = angle;
        m_drive  = drive;
        addRequirements(turret, angle);
    }

    @Override
    public void initialize() {
        m_turret.setFerryMode(true);
        m_turret.configureTrackingPipelineAndFilters();
    }

    @Override
    public void execute() {
        double leadDeg = 0.0;

        if (m_drive != null && m_turret.hasTarget()) {
            double dist = m_turret.getDistanceToTargetM();
            if (dist > 0.0) {
                var state      = m_drive.getState();
                double heading = state.Pose.getRotation().getRadians();
                double vx      = state.Speeds.vxMetersPerSecond;
                double vy      = state.Speeds.vyMetersPerSecond;

                // Convert robot-relative velocity to field frame
                double vx_f = vx * Math.cos(heading) - vy * Math.sin(heading);
                double vy_f = vx * Math.sin(heading) + vy * Math.cos(heading);

                // Unit vector along turret line-of-sight in field frame
                double turretFieldRad = heading + Math.toRadians(m_turret.getAngleDeg());
                double tdx = Math.cos(turretFieldRad);
                double tdy = Math.sin(turretFieldRad);

                // Lateral velocity (perpendicular to line-of-sight)
                double v_lateral   = -vx_f * tdy + vy_f * tdx;
                double tof         = Constants.ShooterTable.getShotTime(dist);
                double lateralDisp = v_lateral * tof;
                leadDeg = Math.toDegrees(Math.atan2(lateralDisp, dist));

                SmartDashboard.putNumber("Ferry/LeadDeg", leadDeg);
            }
        }

        // Use ferry offset + lead compensation
        m_turret.aimAtAprilTag(true, leadDeg);

        // Set hood to fixed ferry angle
        m_angle.setAngleDeg(Constants.FerryConstants.ANGLE_DEG);

        // Publish on-target status
        boolean onTarget = m_turret.hasTarget()
            && Math.abs(m_turret.getTxDeg()) <= Constants.VisionConstants.TX_DEADBAND_DEG * 2.0;
        SmartDashboard.putBoolean("Ferry/OnTarget", onTarget);
        SmartDashboard.putNumber("Ferry/LockedTagID", m_turret.getLockedTagId());
        SmartDashboard.putNumber("Ferry/TurretOffset", m_turret.getFerryOffsetForCurrentTag());
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
        m_turret.setFerryMode(false);
        m_turret.configureTrackingPipelineAndFilters(); // reset to normal tracking tags
        SmartDashboard.putBoolean("Ferry/OnTarget", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
