package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class FeedForwardTest extends CommandBase {
    private Drive drive;
    private double distanceMeters;
    private double speed;

    public FeedForwardTest(Drive drive, double distanceMeters, double speed) {
        setName("FeedForwardTest");
        addRequirements(drive);

        this.drive = drive;
        this.distanceMeters = distanceMeters;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        drive.setBrake();
        drive.resetDriveSensors();
    }

    @Override
    public void execute() {
        double correction = drive.getAbsHeading() * 0.1;
        double moveSpeed = Math.copySign(speed, distanceMeters);
        drive.drivePercent(moveSpeed + correction, moveSpeed - correction);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drivePercent(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getLeftPositionMeters()) > this.distanceMeters
                || Math.abs(drive.getRightPositionMeters()) > this.distanceMeters;
    }
}
