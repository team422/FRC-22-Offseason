package frc.robot.subsystems.swervedrive.swervemodule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    public default SwerveModuleState getState() {
        return new SwerveModuleState(0, new Rotation2d(0));
    }

    public default void rotateVolts() {
    }

    public default void rotateVelocity() {
    }

    public default void translateVolts() {
    }

    public default void translateVelocity() {
    }
}
