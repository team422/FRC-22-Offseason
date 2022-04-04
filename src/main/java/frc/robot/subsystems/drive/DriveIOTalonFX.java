package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class DriveIOTalonFX implements DriveIO {
    private static final double encoderTicksPerRev = 2048.0;
    private static final double encoderTicksPerRad = encoderTicksPerRev / (2 * Math.PI);

    private final double gearingRatio;

    private final WPI_TalonFX leftLeader;
    private final WPI_TalonFX leftFollower;
    private final WPI_TalonFX rightLeader;
    private final WPI_TalonFX rightFollower;

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    public DriveIOTalonFX() {
        switch (Constants.bot) {
            case COMP:
                this.leftLeader = new WPI_TalonFX(7);
                this.leftFollower = new WPI_TalonFX(6);
                this.rightLeader = new WPI_TalonFX(11);
                this.rightFollower = new WPI_TalonFX(5);

                gearingRatio = 6.75;
                break;
            case PRACTICE:
                this.leftLeader = new WPI_TalonFX(4);
                this.leftFollower = new WPI_TalonFX(2);
                this.rightLeader = new WPI_TalonFX(3);
                this.rightFollower = new WPI_TalonFX(1);

                gearingRatio = 1;
                break;
            default:
                throw new RuntimeException("Invalid robot for DriveIOTalonFX!");
        }

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

        leftLeader.configAllSettings(config);
        rightLeader.configAllSettings(config);

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        leftLeader.setInverted(TalonFXInvertType.CounterClockwise);
        leftFollower.setInverted(InvertType.FollowMaster);

        rightLeader.setInverted(TalonFXInvertType.Clockwise);
        rightFollower.setInverted(InvertType.FollowMaster);

        gyro.calibrate();
    }

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec,
            double leftFF, double rightFF) {
        double leftTicksPer100ms = leftRadPerSec * encoderTicksPerRad / 10 * gearingRatio;
        double rightTicksPer100ms = rightRadPerSec * encoderTicksPerRad / 10 * gearingRatio;

        leftLeader.set(ControlMode.Velocity, leftTicksPer100ms, DemandType.ArbitraryFeedForward, leftFF / 12.0);
        rightLeader.set(ControlMode.Velocity, rightTicksPer100ms, DemandType.ArbitraryFeedForward, rightFF / 12.0);
    }

    @Override
    public void setGains(double kP, double kD) {
        leftLeader.config_kP(0, kP);
        leftLeader.config_kD(0, kD);
        rightLeader.config_kP(0, kP);
        rightLeader.config_kD(0, kD);
    }

    @Override
    public void setBrake(boolean brake) {
        if (brake) {
            leftLeader.setNeutralMode(NeutralMode.Brake);
            rightLeader.setNeutralMode(NeutralMode.Brake);
            leftFollower.setNeutralMode(NeutralMode.Brake);
            rightFollower.setNeutralMode(NeutralMode.Brake);
        } else {
            leftLeader.setNeutralMode(NeutralMode.Coast);
            rightLeader.setNeutralMode(NeutralMode.Coast);
            leftFollower.setNeutralMode(NeutralMode.Coast);
            rightFollower.setNeutralMode(NeutralMode.Coast);
        }
    }

    @Override
    public double getLeftPositionRad() {
        return leftLeader.getSelectedSensorPosition(0) / encoderTicksPerRad / gearingRatio;
    }

    @Override
    public double getRightPositionRad() {
        return leftLeader.getSelectedSensorPosition(0) / encoderTicksPerRad / gearingRatio;
    }

    @Override
    public double getLeftVelocityRadPerSec() {
        return leftLeader.getSelectedSensorVelocity(0) / encoderTicksPerRad * 10 / gearingRatio;
    }

    @Override
    public double getRightVelocityRadPerSec() {
        return rightLeader.getSelectedSensorVelocity(0) / encoderTicksPerRad * 10 / gearingRatio;
    }

    @Override
    public double getHeadingRad() {
        // The gyro value is inverted so that a counter-clockwise
        // rotation gives positive (like a polar coordinate system)
        return Units.degreesToRadians(-gyro.getAngle());
    }

    @Override
    public void resetDriveSensors() {
        gyro.reset();
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }
}
