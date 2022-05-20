package frc.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModuleConfig {
    //Module Variables
    private final int idDrive;
    private final int idSteer;
    private final double degOffSet;

    //Drive objects for the Module
    private final WPI_TalonFX drive;
    private TalonFXSensorCollection driveEncoder;

    //Steering objects for the Module (encoder will be analogInput OR canCoder)
    public final WPI_TalonFX steer;
    private TalonFXSensorCollection steerEncoder;

    //PID Vars
    private double kPSteer;
    private double kISteer;
    private double kDSteer;
    private double kFSteer;
    private double kPDrive;
    private double kIDrive;
    private double kDDrive;
    private double kFDrive;

    public SwerveModuleConfig(int idDrive, int idSteer, double degOffSet, double kPSteer,
            double kISteer, double kDSteer, double kFSteer, double kPDrive, double kIDrive, double kDDrive,
            double kFDrive) {
        this.idDrive = idDrive;
        this.idSteer = idSteer;
        this.degOffSet = degOffSet;
        this.kPSteer = kPSteer;
        this.kISteer = kISteer;
        this.kDSteer = kDSteer;
        this.kFSteer = kFSteer;
        this.kPDrive = kPDrive;
        this.kIDrive = kIDrive;
        this.kDDrive = kDDrive;
        this.kFDrive = kFDrive;

        //Motor Config
        drive = new WPI_TalonFX(idDrive);
        steer = new WPI_TalonFX(idSteer);

        //PID Config
        drive.config_kP(0, kPDrive);
        drive.config_kI(0, kIDrive);
        drive.config_kD(0, kDDrive);
        drive.config_kF(0, kFDrive);
        steer.config_kP(0, kPDrive);
        steer.config_kI(0, kIDrive);
        steer.config_kD(0, kDDrive);
        steer.config_kF(0, kFDrive);

        //Encoder Config
        driveEncoder = drive.getSensorCollection();
        steerEncoder = steer.getSensorCollection();
    }

    public void configSteerPID(double kp, double ki, double kd, double kf) {
        steer.config_kP(0, kp);
        steer.config_kI(0, ki);
        steer.config_kD(0, kd);
        steer.config_kF(0, kf);
    }

    public void configDrivePID(double kp, double ki, double kd, double kf) {
        steer.config_kP(0, kp);
        steer.config_kI(0, ki);
        steer.config_kD(0, kd);
        steer.config_kF(0, kf);
    }

    public double getDriveVelocity() {
        return driveEncoder.getIntegratedSensorVelocity();
    }

    public double getAbsoluteEncoderRad() {
        return Math.toRadians(getAngle());
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            steer.set(0);
            drive.set(0);
            return;
        }
        double target = state.angle.getDegrees();
        double Angle = SwerveModuleMath.boundPM180(target - getAngle());
        double pos = steer.getSelectedSensorPosition() + (-Angle) * (Constants.encoderTalonFXTicksPerRev / 360);
        steer.set(ControlMode.Position, pos);
        drive.set(ControlMode.Velocity, state.speedMetersPerSecond);
    }

    public void resetEncoders() {
        driveEncoder.setIntegratedSensorPosition(0, 0);
    }

    public double getAngle() {
        double ang = steerEncoder.getIntegratedSensorPosition();
        ang = ang * 360 / Constants.encoderTalonFXTicksPerRev + degOffSet + 90; //Compassify
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
