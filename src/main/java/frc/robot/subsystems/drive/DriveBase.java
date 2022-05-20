package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {

    private final DriveIO driveIO;

    public DriveBase(DriveIO driveIO) {
        this.driveIO = driveIO;
    }

    @Override
    public void periodic() {
        driveIO.updateOdometry();
    }

    public void setDrivePID(double[] p, double[] i, double[] d, double[] f) {
        driveIO.setDrivePID(p, i, d, f);
    }

    public void setSteerPID(double[] p, double[] i, double[] d, double[] f) {
        driveIO.setDrivePID(p, i, d, f);
    }

    public void setStates(SwerveModuleState[] states) {
        driveIO.setStates(states);
    }

    public void stop() {
        driveIO.stap();
    }

    public double getHeading() {
        return driveIO.getHeading();
    }

    public double getGyroAngle() {
        return driveIO.getGyroAngle();
    }

    public void resetGyro() {
        driveIO.resetGyroAngle();
    }

    public void calibrateGyro() {
        System.out.println("Calibrating Gyro...");
        driveIO.calibrateGyro();
    }
}
