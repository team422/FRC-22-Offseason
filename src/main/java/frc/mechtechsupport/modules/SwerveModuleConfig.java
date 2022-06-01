package frc.mechtechsupport.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.mechtechsupport.util.SwerveModuleMath;
import frc.mechtechsupport.util.TalonFXUtils;
import frc.robot.Constants;

public class SwerveModuleConfig {
    private final double degOffSet;

    //Drive objects for the Module
    private final WPI_TalonFX drive;
    private TalonFXSensorCollection driveEncoder;

    //Steering objects for the Module (encoder will be analogInput OR canCoder)
    private final WPI_TalonFX steer;
    private TalonFXSensorCollection steerEncoder;

    public SwerveModuleConfig(int idDrive, int idSteer, double degOffSet, double kPSteer,
            double kISteer, double kDSteer, double kFSteer, double kPDrive, double kIDrive, double kDDrive,
            double kFDrive) {
        this.degOffSet = degOffSet;

        //Motor Config
        drive = new WPI_TalonFX(idDrive);
        steer = new WPI_TalonFX(idSteer);

        //PID Config
        drive.config_kP(0, kPDrive);
        drive.config_kI(0, kIDrive);
        drive.config_kD(0, kDDrive);
        drive.config_kF(0, kFDrive);
        steer.config_kP(0, kPSteer);
        steer.config_kI(0, kISteer);
        steer.config_kD(0, kDSteer);
        steer.config_kF(0, kFSteer);

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

    public void setSteer(double heading) {
        steer.set(ControlMode.Position,
                TalonFXUtils.wheelDegreesToTicks(SwerveModuleMath.boundPM180(heading), Constants.steerGearRatio));
    }

    public void setDrive(double speed) {
        steer.set(ControlMode.Velocity, speed);
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
        double setPoint = steer.getSelectedSensorPosition() + trueAngle * Constants.encoderTalonFXTicksPerRev / 360.0;
        steer.set(ControlMode.Position, setPoint);
        drive.set(ControlMode.Velocity, isInvert ? -speed : speed);
    }

    public void resetEncoders() {
        driveEncoder.setIntegratedSensorPosition(0, 0);
    }

    public double getAngle() {
        double ang = steerEncoder.getIntegratedSensorPosition();
        ang = ang * 360 / Constants.encoderTalonFXTicksPerRev + degOffSet; //Compassify
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
