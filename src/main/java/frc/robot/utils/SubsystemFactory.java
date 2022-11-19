package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.FullSwerveBase;
import frc.robot.subsystems.SwerveModule;

public class SubsystemFactory {
    private MotorFactory motorFactory;

    public SubsystemFactory(MotorFactory motorFactory) {
        this.motorFactory = motorFactory;
    }

    public SwerveModule createSwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID,
            double turningCANCoderOffsetDegrees, int moduleID) {
        CANSparkMax m_drive_motor = motorFactory.createDrivingMotor(driveMotorID, false);
        CANSparkMax m_turningMotor = motorFactory.createTurningMotor(turningMotorID, false);
        AnalogEncoder m_turningCANCoder = motorFactory.createAnalogEncoder(turningEncoderID);
        RelativeEncoder m_driveEncoder = motorFactory.createDrivingEncoder(m_drive_motor);
        RelativeEncoder m_turningEncoder = motorFactory.createTurningEncoder(m_turningMotor);
        SparkMaxPIDController m_turningController = motorFactory.createTurningController(m_turningMotor);
        SparkMaxPIDController m_driveController = motorFactory.createDrivingController(m_drive_motor);
        return new SwerveModule(m_drive_motor, m_turningMotor, m_turningCANCoder, turningCANCoderOffsetDegrees,
                m_driveEncoder, m_turningEncoder, m_turningController, m_driveController);
    }

    public FullSwerveBase createFullSwerveBase() {

        SwerveModule m_RightFrontSwerveModule = this.createSwerveModule(Constants.DriveConstants.kFrontRightDriveMotor,
                Constants.DriveConstants.kFrontRightTurningMotor, Constants.DriveConstants.kFrontRightEncoder, 0.0, 0);
        SwerveModule m_LeftFrontSwerveModule = this.createSwerveModule(Constants.DriveConstants.kFrontLeftDriveMotor,
                Constants.DriveConstants.kFrontLeftTurningMotor, Constants.DriveConstants.kFrontLeftEncoder, 0.0, 1);
        SwerveModule m_RightRearSwerveModule = this.createSwerveModule(Constants.DriveConstants.kRearRightDriveMotor,
                Constants.DriveConstants.kRearRightTurningMotor, Constants.DriveConstants.kRearRightEncoder, 0.0, 2);
        SwerveModule m_LeftRearSwerveModule = this.createSwerveModule(Constants.DriveConstants.kRearLeftDriveMotor,
                Constants.DriveConstants.kRearLeftTurningMotor, Constants.DriveConstants.kRearLeftEncoder, 0, 3);
        SwerveModule[] m_SwerveModules = new SwerveModule[] { m_LeftFrontSwerveModule, m_RightFrontSwerveModule,
                m_LeftRearSwerveModule, m_RightRearSwerveModule };

        ADXRS450_Gyro m_Gyro = new ADXRS450_Gyro();

        FullSwerveBase m_SwerveBase = new FullSwerveBase(m_SwerveModules, m_Gyro);
        return m_SwerveBase;
    }
}
