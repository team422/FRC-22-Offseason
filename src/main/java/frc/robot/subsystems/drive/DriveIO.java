package frc.robot.subsystems.drive;

public interface DriveIO {
    /**
     * An update method that is called every loop
     */
    public default void update() {
    }

    /**
     * Sets the velocity of the drivetrain motors
     * @param leftRadPerSec Desired angular velocity of the left side
     * @param rightRadPerSec Desired angular velocity of the right side
     * @param leftFF Feedforward output for left side
     * @param rightFF Feedforward output for right side
     */
    public default void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFF, double rightFF) {
    }

    /**
     * Set the P and D gains for the drivetrain (I is not necessary for drivetrain)
     * @param kP P gain to be set
     * @param kD D gain to be set
     */
    public default void setGains(double kP, double kD) {
    }

    /**
     * Sets the idle mode for the drivetrain
     * @param brake Value for the mode (true if brake mode, false if coast mode)
     */
    public default void setBrake(boolean brake) {
    }

    /**
     * Gets the left side encoder value
     * @return Left position, in radians
     */
    public default double getLeftPositionRad() {
        return 0.0;
    }

    /**
     * Gets the right side encoder value
     * @return Right position, in radians
     */
    public default double getRightPositionRad() {
        return 0.0;
    }

    /**
     * Gets the left side velocity encoder value
     * @return Left velocity, in radians per second
     */
    public default double getLeftVelocityRadPerSec() {
        return 0.0;
    }

    /**
     * Gets the right side velocity encoder value
     * @return Right velocity, in radians per second
     */
    public default double getRightVelocityRadPerSec() {
        return 0.0;
    }

    /**
     * Gets the angle of the robot, as calculated by the gyro
     * @return Robot heading, in radians
     */
    public default double getHeadingRad() {
        return 0.0;
    }

    /**
     * Resets all drivetrain sensors (motor encoders and gyro)
     */
    public default void resetDriveSensors() {
    }
}
