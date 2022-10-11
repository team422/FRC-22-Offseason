package frc.robot.subsystems.swervedrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turnMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turnEncoder;

    private final SparkMaxPIDController m_driveController;
    private final SparkMaxPIDController m_turnController;

    public SwerveModuleIOSparkMax(int driveMotorPort, int turnMotorPort) {
        m_driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getEncoder();

        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setIdleMode(IdleMode.kBrake);

        // TODO Set conversion factors here
        // m_driveEncoder.setPositionConversionFactor(factor);
        // m_driveEncoder.setVelocityConversionFactor(factor);

        // m_turnEncoder.setPositionConversionFactor(factor);

        m_driveController = m_driveMotor.getPIDController();
        m_turnController = m_turnMotor.getPIDController();

        // TODO Set PID values here
        // m_driveController.setP(gain);
        // m_driveController.setI(gain);
        // m_driveController.setD(gain);

        // m_turnController.setP(gain);
        // m_turnController.setI(gain);
        // m_turnController.setD(gain);
    }

    public SwerveModuleState getState() {
        double angle = (m_turnEncoder.getPosition() % 360 + 360) % 360;
        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(angle * Math.PI / 180));
    }

    public void resetTurnEncoder() {
        m_turnEncoder.setPosition(0.0);
    }

    public void resetDriveEncoder() {
        m_driveEncoder.setPosition(0.0);
    }

    public double getDriveDistanceMeters() {
        return m_driveEncoder.getPosition();
    }

    public void setDesiredState(SwerveModuleState state) {
        //TODO Finish setDesiredState code
    }

    public void setOpenLoopState(SwerveModuleState state) {
        //TODO Finish setOpenLoopState code
    }

    public double deltaAdjustedAngle(double targetAngle, double currentAngle) {
        // Needs to be between -180 and 180 as opposed to being between 0 and 360
        return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }
}
