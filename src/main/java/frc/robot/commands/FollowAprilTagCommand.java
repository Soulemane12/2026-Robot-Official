package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.generated.TunerConstants;

/**
 * Command that makes the robot follow an AprilTag.
 * The robot will:
 * - Always face the AprilTag (rotation control using TX)
 * - Maintain a set distance from the AprilTag (forward/back control using TY or area)
 * - Center horizontally with the AprilTag (strafe control using TX)
 *
 * Hold a board with an AprilTag and move it around - the robot will follow!
 */
public class FollowAprilTagCommand extends Command {
    private final VisionSubsystem m_vision;
    private final CommandSwerveDrivetrain m_swerve;
    private final SwerveRequest.RobotCentric m_driveRequest;

    // Speed limits (fraction of max speed)
    private final double MaxSpeed;
    private final double MaxAngularRate;

    // Target setpoints
    private final double targetTY;      // Target TY value (controls distance)
    private final double targetAngle;   // Target angle (usually 0 to face the tag head-on)

    // PID gains - TUNE THESE FOR YOUR ROBOT!
    private static final double kP_rotation = 0.02;     // Proportional gain for rotation (facing the tag)
    private static final double kP_forward = 0.06;      // Proportional gain for forward/backward movement
    private static final double kP_strafe = 0.04;       // Proportional gain for left/right strafing

    // Tolerances
    // TODO: figure out why unused
    // private static final double angleTolerance = 2.0;   // Degrees tolerance for rotation
    // private static final double distanceTolerance = 1.0; // TY tolerance for distance

    // Lost target handling
    private double lastValidTX = 0.0;
    private double lastValidTY = 0.0;
    private double lastStrafeAdjust = 0.0;
    private final Timer lostTargetTimer = new Timer();
    private static final double lostTargetTimeout = 0.5; // Seconds before giving up on lost target

    /**
     * Creates a new FollowAprilTagCommand.
     *
     * @param vision The vision subsystem
     * @param swerve The swerve drivetrain
     * @param targetTY The target TY value to maintain (controls distance)
     *                 Higher TY = closer to the tag
     *                 Start with a value like 0.0 and adjust based on testing
     * @param targetAngle The target angle offset (usually 0 to face head-on)
     */
    public FollowAprilTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain swerve,
                                  double targetTY, double targetAngle) {
        m_vision = vision;
        m_swerve = swerve;
        this.targetTY = targetTY;
        this.targetAngle = targetAngle;

        // Set up the drive request for robot-centric control
        m_driveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.05)
            .withRotationalDeadband(0.05)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        // Limit speed to 50% for safety during tracking
        MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5;
        MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.5;

        addRequirements(m_vision, m_swerve);
    }

    /**
     * Creates a FollowAprilTagCommand with default values.
     * targetTY = 0 (robot will try to maintain the distance where TY = 0)
     * targetAngle = 0 (robot will face the tag head-on)
     */
    public FollowAprilTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain swerve) {
        this(vision, swerve, 0.0, 0.0);
    }

    private int debugCounter = 0;

    @Override
    public void initialize() {
        System.out.println("===========================================");
        System.out.println("FollowAprilTagCommand started - looking for AprilTag...");
        System.out.println("Using Limelight: \"" + m_vision.getLimelightName() + "\"");
        System.out.println("===========================================");
        lostTargetTimer.reset();
        lostTargetTimer.start();
        lastValidTX = 0.0;
        lastValidTY = 0.0;
        lastStrafeAdjust = 0.0;
        debugCounter = 0;
    }

    @Override
    public void execute() {
        // Get current vision data
        double currentTX = m_vision.getTargetTX();
        double currentTY = m_vision.getTargetTY();
        double currentTA = m_vision.getTargetArea();
        double tagID = m_vision.getTagID();
        boolean hasTarget = m_vision.hasValidTarget();

        // Debug output every 25 cycles (about twice per second)
        debugCounter++;
        if (debugCounter >= 25) {
            debugCounter = 0;
            System.out.println("--- FollowAprilTag Debug ---");
            System.out.println("  hasTarget: " + hasTarget);
            System.out.println("  TX: " + currentTX + ", TY: " + currentTY + ", TA: " + currentTA);
            System.out.println("  Tag ID: " + tagID);
            if (!hasTarget) {
                System.out.println("  >>> NO TARGET DETECTED! Check:");
                System.out.println("      1. Is Limelight name correct? Using: \"" + m_vision.getLimelightName() + "\"");
                System.out.println("      2. Is Limelight in AprilTag pipeline mode?");
                System.out.println("      3. Is there an AprilTag visible to the camera?");
            }
        }

        // Handle lost target - use last valid values for a short time
        if (hasTarget && (currentTX != 0.0 || currentTY != 0.0)) {
            lastValidTX = currentTX;
            lastValidTY = currentTY;
            lostTargetTimer.reset();
        } else if (lostTargetTimer.get() < lostTargetTimeout) {
            // Use last valid values if target recently lost
            currentTX = lastValidTX;
            currentTY = lastValidTY;
        } else {
            // Target lost for too long - stop moving
            currentTX = 0.0;
            currentTY = 0.0;
        }

        // Calculate control outputs

        // 1. ROTATION: Make the robot face the AprilTag
        // TX is the horizontal offset - we want it to be targetAngle (usually 0)
        double rotationError = currentTX - targetAngle;
        double rotationOutput = -rotationError * kP_rotation * MaxAngularRate;

        // 2. FORWARD/BACKWARD: Maintain distance from the AprilTag
        // TY is the vertical offset - we want it to be targetTY
        double distanceError = targetTY - currentTY;
        double forwardOutput = distanceError * kP_forward * MaxSpeed;

        // 3. STRAFE: Keep the AprilTag centered (optional - can remove if just want to face it)
        // Using TX to strafe - smoother tracking
        double strafeError = -currentTX;
        double strafeOutput = strafeError * kP_strafe * MaxSpeed;

        // Smooth the strafe output to prevent jerky movement
        strafeOutput = (strafeOutput + lastStrafeAdjust) / 2.0;
        lastStrafeAdjust = strafeOutput;

        // Apply the control to the drivetrain
        m_swerve.setControl(
            m_driveRequest
                .withVelocityX(forwardOutput)    // Forward/backward
                .withVelocityY(strafeOutput)     // Left/right strafe
                .withRotationalRate(rotationOutput)  // Rotation to face target
        );

        // Debug output to SmartDashboard
        SmartDashboard.putNumber("AprilTag/TX", currentTX);
        SmartDashboard.putNumber("AprilTag/TY", currentTY);
        SmartDashboard.putNumber("AprilTag/RotationOutput", rotationOutput);
        SmartDashboard.putNumber("AprilTag/ForwardOutput", forwardOutput);
        SmartDashboard.putNumber("AprilTag/StrafeOutput", strafeOutput);
        SmartDashboard.putBoolean("AprilTag/HasTarget", hasTarget);
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        // You could add logic here to finish when "aligned" if desired
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FollowAprilTagCommand ended" + (interrupted ? " (interrupted)" : ""));

        // Stop the robot
        m_swerve.setControl(
            m_driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );

        lostTargetTimer.stop();
    }
}
