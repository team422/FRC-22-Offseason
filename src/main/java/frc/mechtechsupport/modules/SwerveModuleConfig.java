package frc.mechtechsupport.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.mechtechsupport.util.SwerveModuleMath;
import frc.mechtechsupport.util.TalonFXUtils;
import frc.robot.Constants;

public class SwerveModuleConfig {

    //Drive objects for the Module
    private final WPI_TalonFX drive;

    //Steering objects for the Module (encoder will be analogInput OR canCoder)
    private final CANSparkMax steer;
    private final CANCoder steerAbsEncoder;
    private final RelativeEncoder steerEncoder;
    private final SparkMaxPIDController steerController;

    public SwerveModuleConfig(int idDrive, int idSteer, int steerEncoderID, double kPSteer,
            double kISteer, double kDSteer, double kFSteer, double kPDrive, double kIDrive, double kDDrive,
            double kFDrive) {

        //Motor Config
        drive = new WPI_TalonFX(idDrive);
        steer = new CANSparkMax(idSteer, MotorType.kBrushless);

        //Encoder Config
        this.steerEncoder = steer.getEncoder();
        steerEncoder.setPositionConversionFactor(2048);
        this.steerAbsEncoder = new CANCoder(steerEncoderID);
        steerAbsEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        //PID Config
        this.steerController = steer.getPIDController();
        drive.config_kP(0, kPDrive);
        drive.config_kI(0, kIDrive);
        drive.config_kD(0, kDDrive);
        drive.config_kF(0, kFDrive);
        steerController.setP(kPSteer);
        steerController.setI(kISteer);
        steerController.setD(kDSteer);
        steerController.setFF(kFSteer);

    }

    public void configSteerPID(double kp, double ki, double kd, double kf) {
        steerController.setP(kp);
        steerController.setI(ki);
        steerController.setD(kd);
        steerController.setFF(kf);
    }

    public void configDrivePID(double kp, double ki, double kd, double kf) {
        drive.config_kP(0, kp);
        drive.config_kI(0, ki);
        drive.config_kD(0, kd);
        drive.config_kF(0, kf);
    }

    public double getDriveVelocity() {
        return drive.getSelectedSensorVelocity();
    }

    public double getAbsoluteEncoderRad() {
        return Math.toRadians(getAngle());
    }

    public void setSteer(double heading) {
        steerController.setReference(
                TalonFXUtils.wheelDegreesToTicks(SwerveModuleMath.boundPM180(heading), Constants.steerGearRatio),
                ControlType.kPosition);
        //needs to have correct ticks to revolution put in or it won't work
    }

    public void setDrive(double speed) {
        drive.set(ControlMode.Velocity, speed);
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            steer.set(0);
            drive.set(0);
            return;
        }
        double target = state.angle.getDegrees();
        optomizedDrive(target, state.speedMetersPerSecond);
    }

    private void optomizedDrive(double targetAng, double speed) {
        double deltaAngle = SwerveModuleMath.boundPM180(targetAng - getAngle());
        double invDeltaAngle = SwerveModuleMath.boundPM180(targetAng - getAngle() + 180);
        double optomizedAngleMagnitude = Math.min(Math.abs(deltaAngle), Math.abs(invDeltaAngle));
        double trueAngle = Math.abs(deltaAngle) == optomizedAngleMagnitude ? deltaAngle : invDeltaAngle;
        boolean isInvert = Math.abs(deltaAngle) == optomizedAngleMagnitude ? false : true;
        double setPoint = steerEncoder.getPosition() + trueAngle * Constants.encoderTalonFXTicksPerRev / 360.0;
        steerController.setReference(setPoint, ControlType.kPosition);
        drive.set(ControlMode.Velocity, isInvert ? -speed : speed);
    }

    public double getAngle() {
        double ang = steerAbsEncoder.getAbsolutePosition();
        ang = ang * 360 / Constants.encoderTalonFXTicksPerRev; //Compassify
        if (ang > 360) {
            ang -= 360;
        }
        return ang;
    }

    public void stop() {
        drive.set(0);
        steer.set(0);
    }
}
