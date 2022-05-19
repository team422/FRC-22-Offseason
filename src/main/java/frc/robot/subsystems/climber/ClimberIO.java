package frc.robot.subsystems.climber;

public interface ClimberIO {

    public default void setPID(double kf, double kp, double ki, double kd, boolean isPositionPID) {
    }

    public default boolean isAtSetPoint() {
        return false;
    }

    public default double getSetPoint() {
        return 0.0;
    }

    public default void setBrakeMode(boolean enable) {
    }

    public default void setVelocity(double velocity) {
    }

    public default void setPercentPower(double power) {
    }

    public default void setTargetPoint(double encoderValue) {
    }

    public default void setRightPercent(double power) {
    }

    public default void setLeftPercent(double power) {
    }

    public default void setPosition(double position) {
    }

    public default double getEncoderPosition() {
        return 0.0;
    }
}
