package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The drive base of the robot
 */
public class Drive extends SubsystemBase {
    private final double wheelRadiusMeters;
    private final double maxVelocityMetersPerSec;
    private final SimpleMotorFeedforward leftModel, rightModel;

    private final DriveIO io;
    private final DifferentialDriveOdometry odometry;
    private Rotation2d heading;

    /**
     * Instantiates a new instance of the drive subsystem (only one should be instantiated)
     * @param io Drive subsystem IO
     */
    public Drive(DriveIO io) {
        this.io = io;
        io.resetDriveSensors();

        switch (Constants.bot) {
            case COMP:
                wheelRadiusMeters = Units.inchesToMeters(2);
                maxVelocityMetersPerSec = Units.feetToMeters(16.5);

                leftModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0); // TODO Change this
                rightModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0); // TODO Change this
                break;
            case PRACTICE:
                wheelRadiusMeters = Units.inchesToMeters(2);
                maxVelocityMetersPerSec = Units.feetToMeters(16.5); // Confirm this

                leftModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0); // TODO Change this
                rightModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0); // TODO Change this
                break;
            default:
                throw new RuntimeException("Attempting to instantiate drive subsystem for invalid robot!");
        }

        this.heading = new Rotation2d(io.getHeadingRad());
        this.odometry = new DifferentialDriveOdometry(heading);

        SmartDashboard.putNumber("DriveLeftVel", 0);
        SmartDashboard.putNumber("DriveRightVel", 0);
    }

    public double getLeftPositionMeters() {
        return io.getLeftPositionRad() * wheelRadiusMeters;
    }

    public double getRightPositionMeters() {
        return io.getRightPositionRad() * wheelRadiusMeters;
    }

    public double getLeftVelocityMetersPerSec() {
        return io.getRightVelocityRadPerSec() * wheelRadiusMeters;
    }

    public double getRightVelocityMetersPerSec() {
        return io.getRightVelocityRadPerSec() * wheelRadiusMeters;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(),
                getRightVelocityMetersPerSec());
    }

    public double getAbsHeading() {
        return io.getHeadingRad();
    }

    public Rotation2d getHeading() {
        return heading;
    }

    @Override
    public void periodic() {
        io.update();
        heading = new Rotation2d(io.getHeadingRad());

        // Update odometry
        odometry.update(heading, getLeftPositionMeters(), getRightPositionMeters());
    }

    public void drivePercent(double leftPercent, double rightPercent) {
        driveVelocity(leftPercent * maxVelocityMetersPerSec,
                rightPercent * maxVelocityMetersPerSec);
    }

    public void driveVelocity(double leftVelocityMetersPerSec, double rightVelocityMetersPerSec) {
        double leftVelocityRadPerSec = leftVelocityMetersPerSec / wheelRadiusMeters;
        double rightVelocityRadPerSec = rightVelocityMetersPerSec / wheelRadiusMeters;
        double leftFF = leftModel.calculate(leftVelocityMetersPerSec);
        double rightFF = rightModel.calculate(rightVelocityMetersPerSec);

        io.setVelocity(leftVelocityRadPerSec, rightVelocityRadPerSec,
                leftFF, rightFF);
    }

    public void setGains(double kP, double kD) {
        io.setGains(kP, kD);
    }

    public void setBrake() {
        io.setBrake(true);
    }

    public void setCoast() {
        io.setBrake(false);
    }

    public void resetDriveSensors() {
        io.resetDriveSensors();
    }
}
