package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ClimberIOFalcon implements ClimberIO {
    private static final double encoderTicksPerRev = 2048.0;

    public static final int leftClimberPort = 15;
    public static final int rightClimberPort = 16;

    private final WPI_TalonFX leftClimberMotor;
    private final WPI_TalonFX rightClimberMotor;

    private TalonFXSensorCollection encoderBoi;

    public ClimberIOFalcon() {
        this.leftClimberMotor = new WPI_TalonFX(leftClimberPort);
        this.rightClimberMotor = new WPI_TalonFX(rightClimberPort);
        rightClimberMotor.setInverted(true);
        encoderBoi = leftClimberMotor.getSensorCollection();
    }

    @Override
    public void setPID(double kf, double kp, double ki, double kd) {
        leftClimberMotor.config_kF(0, kf);
        leftClimberMotor.config_kP(0, kp);
        leftClimberMotor.config_kI(0, ki);
        leftClimberMotor.config_kD(0, kd);

        rightClimberMotor.config_kF(0, kf);
        rightClimberMotor.config_kP(0, kp);
        rightClimberMotor.config_kI(0, ki);
        rightClimberMotor.config_kD(0, kd);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
        leftClimberMotor.setNeutralMode(mode);
        rightClimberMotor.setNeutralMode(mode);
    }

    @Override
    public void setVelocity(double velocity) {
        leftClimberMotor.set(ControlMode.Velocity, velocity);
        rightClimberMotor.set(ControlMode.Velocity, velocity);
    }

    @Override
    public void setPercentPower(double power) {
        leftClimberMotor.set(ControlMode.PercentOutput, power);
        rightClimberMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void setTargetPoint(double encoderValue) {
        leftClimberMotor.set(ControlMode.Position, encoderValue);
        rightClimberMotor.set(ControlMode.Position, encoderValue);
    }

    @Override
    public void setRightPercent(double power) {
        rightClimberMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void setLeftPercent(double power) {
        leftClimberMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public double getEncoderPosition() {
        return encoderBoi.getIntegratedSensorPosition();
    }
}
