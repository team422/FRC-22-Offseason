package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    /**
     * Gets the state of this swerve module
     */
    SwerveModuleState getState();

    /**
     * Resets the turn encoder
     */
    void resetTurnEncoder();

    /**
     * Resets the drive encoder
     */
    void resetDriveEncoder();

    /**
     * Resets all encoders
     */
    public default void resetEncoders() {
        resetTurnEncoder();
        resetDriveEncoder();
    }

    /**
     * Gets the drive encoder value
     * 
     * @return The encoder value of the drive motor in meters
     */
    double getDriveDistanceMeters();

    /**
     * Sets the (closed loop) desired state of this swerve module
     * 
     * @param state Desired SwerveModuleState with speed and angle
     */
    void setDesiredState(SwerveModuleState state);

    /**
     * Sets the (open loop) desried state of this swerve module
     * 
     * @param state
     */
    void setOpenLoopState(SwerveModuleState state);

    /**
     * Calculates the angle between the target and current angles
     * 
     * @param targetAngle The degree measure of the target angle
     * @param currentAngle The degree measure of the current angle
     * @return The degree measure of the angle between the target and current angles
     */
    double deltaAdjustedAngle(double targetAngle, double currentAngle);
}
