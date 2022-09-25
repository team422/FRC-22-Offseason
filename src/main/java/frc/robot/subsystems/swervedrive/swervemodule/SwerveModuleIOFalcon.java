package frc.robot.subsystems.swervedrive.swervemodule;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX steerMotor;

    public SwerveModuleIOFalcon(int drivePort, int steerPort) {
        driveMotor = new WPI_TalonFX(drivePort);
        steerMotor = new WPI_TalonFX(steerPort);
    }
}
