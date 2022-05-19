package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ClimberIOFalcon implements ClimberIO {
    private static final double encoderTicksPerRev = 2048.0;
    private static double defaultError = 10; //placeholder allowed error

    public static final int leftClimberPort = 15;
    public static final int rightClimberPort = 16;
    private static double setPoint = 0;
    private static boolean isClimberInPositionMode = true;

    private final WPI_TalonFX leftClimberMotor;
    private final WPI_TalonFX rightClimberMotor;

    private TalonFXSensorCollection encoderBoi;

    public ClimberIOFalcon() {
        this.leftClimberMotor = new WPI_TalonFX(leftClimberPort);
        this.rightClimberMotor = new WPI_TalonFX(rightClimberPort);
        rightClimberMotor.setInverted(true);
        encoderBoi = leftClimberMotor.getSensorCollection();
    }

    /**
     * sets PID loop for the climber
     * 
     * @param kf the feed forwards value of the PID loop
     * @param kp the position feed value of the PID loop
     * @param ki the integral feed value of the PID loop
     * @param kd the derivative feed value of the PID loop
     * @param isPositionPID boolean that lets you know if the PID loop is for position (true) or velocity (false). Used in other commands to overwrite the existing PID loop
     */
    @Override
    public void setPID(double kf, double kp, double ki, double kd, boolean isPositionPID) {
        leftClimberMotor.config_kF(0, kf);
        leftClimberMotor.config_kP(0, kp);
        leftClimberMotor.config_kI(0, ki);
        leftClimberMotor.config_kD(0, kd);

        rightClimberMotor.config_kF(0, kf);
        rightClimberMotor.config_kP(0, kp);
        rightClimberMotor.config_kI(0, ki);
        rightClimberMotor.config_kD(0, kd);

        isClimberInPositionMode = isPositionPID;
    }

    /**
     * Position PID loop has an error/acceptable range of values. If the motor is within this expected range (ie. setpoint +- error), it will start a count down to see if it settles. (ex. See ClimberPIDPos.java)
     * @param error the default error/acceptable range of values
     */
    @Override
    public void setDefaultPosPIDError(double error) {
        defaultError = error;
    }

    /**
     * Returns a boolean to see if the motor is currently within the setpoint +- default error
     * @return is the motor within default range of the set point
     */
    @Override
    public boolean isAtSetPoint() {
        double currentPosition = encoderBoi.getIntegratedSensorPosition();
        return Math.abs(currentPosition - setPoint) <= defaultError;
    }

    /**Returns a boolean to see if the motor is currently within the setpoint +- an error that is not the default error
     * @param error the non default error value that you want to use
     * @return is the motor within acceptable range of the set point
    */
    @Override
    public boolean isAtSetPoint(double error) {
        double currentPosition = encoderBoi.getIntegratedSensorPosition();
        return Math.abs(currentPosition - setPoint) <= error;
    }

    /**
     * Note: Returns setpoint regardless of velocity or position loop, this method can be used for both.
     * @return current Set Point for the PID Loop
     */
    @Override
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * @return is the PID loop in position (true) or velocity (false) mode
     */
    @Override
    public boolean isInPosPIDMode() {
        return isClimberInPositionMode;
    }

    /**
     * sets brake mode (true = brake mode, false = coast mode)
     */
    @Override
    public void setBrakeMode(boolean enable) {
        NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
        leftClimberMotor.setNeutralMode(mode);
        rightClimberMotor.setNeutralMode(mode);
    }

    /**
     * sets the motors to a velocity
     * MAKE SURE YOU ARE IN VELOCITY PID MODE
     * @param velocity target velocity
     */
    @Override
    public void setVelocity(double velocity) {
        leftClimberMotor.set(ControlMode.Velocity, velocity);
        rightClimberMotor.set(ControlMode.Velocity, velocity);
        setPoint = velocity;
    }

    /**
     * sets the motors to a power
     * @param power amount of power that the motor can draw as a decimal of its original power (setPercentPower(0.9); is 90% power)
     */
    @Override
    public void setPercentPower(double power) {
        leftClimberMotor.set(ControlMode.PercentOutput, power);
        rightClimberMotor.set(ControlMode.PercentOutput, power);
    }

    /**
     * sets the Motor to a position set point
     * MAKE SURE YOU ARE IN POSITION PID MODE
     * @param encoderValue the encoder value that you want to go to (IMPORTANT: DO NOT SET IT TO AN INCH OR METER VALUE)
     */
    @Override
    public void setTargetPoint(double encoderValue) {
        leftClimberMotor.set(ControlMode.Position, encoderValue);
        rightClimberMotor.set(ControlMode.Position, encoderValue);
        setPoint = encoderValue;
    }

    @Override
    public void setRightPercent(double power) {
        rightClimberMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void setLeftPercent(double power) {
        leftClimberMotor.set(ControlMode.PercentOutput, power);
    }

    /**
     * @return returns the current encoder position of the left arm.
     */
    @Override
    public double getEncoderPosition() {
        return encoderBoi.getIntegratedSensorPosition();
    }
}
