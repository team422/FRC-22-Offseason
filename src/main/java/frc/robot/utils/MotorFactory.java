package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.ModuleConstants;

// 

public class MotorFactory {
    public CANSparkMax createDrivingMotor(int id, boolean inverted) {
        CANSparkMax motor = new CANSparkMax(id, CANSparkMax.MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.restoreFactoryDefaults();
        motor.setInverted(inverted);
        motor.enableVoltageCompensation(12); // this should really be a constant

        return motor;
    }

    public CANSparkMax createTurningMotor(int id, boolean inverted) {
        CANSparkMax motor = new CANSparkMax(id, CANSparkMax.MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.restoreFactoryDefaults();
        motor.setInverted(inverted);
        return motor;
    }

    public static void createDriveMotor() {

        // m_driveMotor.setIdleMode(IdleMode.kCoast);
        // m_turningMotor.setIdleMode(IdleMode.kBrake); // this should be set in the other function

        // // m_driveEncoder returns RPM by default. Use setVelocityConversionFactor() to
        // // convert that to meters per second.
        // m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveConversionFactor / 60.0);
        // m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveConversionFactor);

        // this.m_driveEncoder.setPositionConversionFactor(360.0 / ModuleConstants.kTurnPositionConversionFactor);

        // m_turningController = m_turningMotor.getPIDController();
        // m_driveController = m_driveMotor.getPIDController();

        // m_driveMotor.enableVoltageCompensation(12);

        //m_turningCANCoder.setDistancePerRotation(turningCANCoderOffsetDegrees);
        // m_turningCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        // m_turningCANCoder.setPosition(0);

        //        m_CANCoderOffset = Rotation2d.fromDegrees(turningCANCoderOffsetDegrees);

        // m_driveMotor.setIdleMode(IdleMode.kBrake);
        // m_turningMotor.setIdleMode(IdleMode.kCoast);
    }

    public RelativeEncoder createDrivingEncoder(CANSparkMax driveMotor) {
        RelativeEncoder driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveConversionFactor);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveConversionFactor / 60.0);
        return driveEncoder;
    }

    public RelativeEncoder createTurningEncoder(CANSparkMax turningMotor) {
        RelativeEncoder turningEncoder = turningMotor.getEncoder();
        turningEncoder.setPositionConversionFactor(360.0 / ModuleConstants.kTurnPositionConversionFactor); // i dont really know what this does
        return turningEncoder;
    }

    public SparkMaxPIDController createTurningController(CANSparkMax turningMotor) {
        SparkMaxPIDController turningController = turningMotor.getPIDController();
        turningController.setP(ModuleConstants.kTurningP);
        turningController.setI(ModuleConstants.kTurningI);
        turningController.setD(ModuleConstants.kTurningD);
        return turningController;
    }

    public SparkMaxPIDController createDrivingController(CANSparkMax turningMotor) {
        SparkMaxPIDController turningController = turningMotor.getPIDController();
        turningController.setP(ModuleConstants.kTurningP);
        turningController.setI(ModuleConstants.kTurningI);
        turningController.setD(ModuleConstants.kTurningD);
        return turningController;
    }

    public AnalogEncoder createAnalogEncoder(int turningChannel) {
        AnalogEncoder turningCANCoder = new AnalogEncoder(turningChannel);
        return turningCANCoder;

    }
    // pubilc AnalogEncoder createAnalogEncoder(int turningCANCoderChannel) {
    //     AnalogEncoder turningCANCoder = new AnalogEncoder(turningCANCoderChannel);
    //     turningCANCoder.setDistancePerRotation(turningCANCoderOffsetDegrees);
    //     return turningCANCoder;
    // }

}
