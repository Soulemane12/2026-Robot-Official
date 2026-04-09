package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Ferry mode auto-aim command.
 * Uses vision tracking with alliance-specific tag filtering and tag-specific turret offsets.
 * Sets hood to fixed ferry angle.
 */
public class FerryAutoAimCommand extends Command {
    private final TurretSubsystem m_turret;
    private final ShooterAngleSubsystem m_angle;

    public FerryAutoAimCommand(TurretSubsystem turret, ShooterAngleSubsystem angle) {
        m_turret = turret;
        m_angle  = angle;
        addRequirements(turret, angle);
    }

    @Override
    public void initialize() {
        m_turret.setFerryMode(true);
        m_turret.configureTrackingPipelineAndFilters();
    }

    @Override
    public void execute() {
        // Use ferry offset mode for turret aiming
        m_turret.aimAtAprilTag(true);

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
