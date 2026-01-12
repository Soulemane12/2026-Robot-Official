package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command that aligns the robot to a reef AprilTag for scoring.
 * Supports both left and right side scoring positions.
 *
 * The robot will:
 * - Position itself at the correct distance from the reef tag (X-axis)
 * - Align horizontally for left or right side scoring (Y-axis)
 * - Rotate to face the tag properly (rotation)
 */
public class AlignToReefTagRelative extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.RobotCentric driveRequest;
    private double tagID = -1;

    /**
     * Creates a new AlignToReefTagRelative command.
     *
     * @param isRightScore true for right side scoring position, false for left side
     * @param drivetrain The swerve drivetrain subsystem
     */
    public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivetrain) {
        // Initialize PID controllers with gains from Constants
        xController = new PIDController(Constants.X_APRILTAG_ALIGNMENT_P, 0.0, 0);  // Vertical movement
        yController = new PIDController(Constants.Y_APRILTAG_ALIGNMENT_P, 0.0, 0);  // Horizontal movement
        rotController = new PIDController(Constants.ROT_APRILTAG_ALIGNMENT_P, 0, 0);  // Rotation

        this.isRightScore = isRightScore;
        this.drivetrain = drivetrain;

        // Set up the drive request for robot-centric control
        driveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.0)
            .withRotationalDeadband(0.0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Initialize timers
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        // Configure rotation controller
        rotController.setSetpoint(Constants.ROT_SETPOINT_APRILTAG_ALIGNMENT);
        rotController.setTolerance(Constants.ROT_TOLERANCE_APRILTAG_ALIGNMENT);

        // Configure X controller (distance)
        xController.setSetpoint(Constants.X_SETPOINT_APRILTAG_ALIGNMENT);
        xController.setTolerance(Constants.X_TOLERANCE_APRILTAG_ALIGNMENT);

        // Configure Y controller (left/right positioning)
        // Positive setpoint for right side, negative for left side
        yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_APRILTAG_ALIGNMENT : -Constants.Y_SETPOINT_APRILTAG_ALIGNMENT);
        yController.setTolerance(Constants.Y_TOLERANCE_APRILTAG_ALIGNMENT);

        // Lock onto the first tag we see
        tagID = LimelightHelpers.getFiducialID("");

        System.out.println("===========================================");
        System.out.println("AlignToReefTagRelative started");
        System.out.println("Scoring side: " + (isRightScore ? "RIGHT" : "LEFT"));
        System.out.println("Target Tag ID: " + tagID);
        System.out.println("===========================================");
    }

    @Override
    public void execute() {
        // Check if we can see the target tag
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
            // Reset the "don't see tag" timer since we see it
            this.dontSeeTagTimer.reset();

            // Get 3D position relative to the AprilTag
            // positions[0] = X (left/right from tag's perspective)
            // positions[2] = Z (distance forward/back)
            // positions[4] = Yaw (rotation)
            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");

            // Debug output
            SmartDashboard.putNumber("ReefAlign/X_Position", positions[2]);
            SmartDashboard.putNumber("ReefAlign/Y_Position", positions[0]);
            SmartDashboard.putNumber("ReefAlign/Yaw", positions[4]);

            // Calculate control outputs using PID
            double xSpeed = xController.calculate(positions[2]);
            SmartDashboard.putNumber("ReefAlign/XSpeed", xSpeed);

            double ySpeed = -yController.calculate(positions[0]);
            double rotValue = -rotController.calculate(positions[4]);

            // Apply control to the drivetrain
            drivetrain.setControl(
                driveRequest
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(rotValue)
            );

            // Check if all controllers are at their setpoints
            if (!rotController.atSetpoint() ||
                !yController.atSetpoint() ||
                !xController.atSetpoint()) {
                // Not aligned yet, reset the stop timer
                stopTimer.reset();
            }

            // Debug output for setpoint status
            SmartDashboard.putBoolean("ReefAlign/X_AtSetpoint", xController.atSetpoint());
            SmartDashboard.putBoolean("ReefAlign/Y_AtSetpoint", yController.atSetpoint());
            SmartDashboard.putBoolean("ReefAlign/Rot_AtSetpoint", rotController.atSetpoint());

        } else {
            // No target visible - stop moving
            drivetrain.setControl(
                driveRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
        }

        SmartDashboard.putNumber("ReefAlign/StopTimer", stopTimer.get());
        SmartDashboard.putNumber("ReefAlign/DontSeeTagTimer", dontSeeTagTimer.get());
        SmartDashboard.putBoolean("ReefAlign/HasTarget", LimelightHelpers.getTV(""));
        SmartDashboard.putNumber("ReefAlign/CurrentTagID", LimelightHelpers.getFiducialID(""));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AlignToReefTagRelative ended " + (interrupted ? "(interrupted)" : "(completed)"));

        // Stop the robot
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        // Command finishes when:
        // 1. Robot doesn't see the tag for too long (timeout)
        // 2. Robot is aligned within tolerances for the required duration
        return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
            stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
    }
}
