package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface DriveIO {

    public default void setDrivePID(double[] Pcontainer, double[] Icontainer, double[] Dcontainer,
            double[] Fcontainer) {
    }

    public default void setSteerPID(double[] Pcontainer, double[] Icontainer, double[] Dcontainer,
            double[] Fcontainer) {
    }

    /** Gets gyro value */
    public default double getGyroAngle() {
        return 0.0;
    }

    /** Resets gyro value */
    public default void resetGyroAngle() {
    }

    public default void calibrateGyro() {
    }

    /** Gets gyro rate */
    public default double getGyroRate() {
        return 0.0;
    }

    public default double getHeading() {
        return 0.0;
    }

    /**I do not trust this I have no clue... I read how to do odometry for like 1 hr */
    public default void updateOdometry() {
    }

    public default Pose2d getPose() {
        return null;
    }

    public default void stap() {
    }

    public default void setStates(SwerveModuleState[] stateContainer) {
    }
}
