package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class FullSwerveBase extends SubsystemBase {
    /**
     * Creates a new FullSwerveBase.
     */
    // these should be in the order front left, front right, back left, back right
    SwerveModule m_swerveModules[] = new SwerveModule[4];
    Gyro m_gyro;
    SwerveDriveOdometry m_odometry;

    //target pose and controller
    Pose2d m_targetPose;
    PIDController m_thetaController = new PIDController(1.0, 0.0, 0.05);

    public FullSwerveBase(SwerveModule[] m_swerveModules, ADXRS450_Gyro gyro) {
        // Setting up all the modules
        this.m_swerveModules = m_swerveModules;
        for (SwerveModule module : m_swerveModules) {
            module.resetDistance();
            // module.syncTurningEncoders();
        }
        // m_targetPose = m_odometry.getPoseMeters();
        m_thetaController.reset();
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.m_gyro = gyro;
        m_gyro.calibrate();
        m_gyro.reset();

        // odometry stuff
        m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, this.getHeading());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(this.getHeading(), m_swerveModules[0].getState(), m_swerveModules[1].getState(),
                m_swerveModules[2].getState(), m_swerveModules[3].getState());

        //This was literally all copy paste, these should be good for debugging
        SmartDashboard.putNumber("Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("currentX", getPose().getX());
        SmartDashboard.putNumber("currentY", getPose().getY());
        SmartDashboard.putNumber("currentAngle", getPose().getRotation().getRadians());
        // SmartDashboard.putNumber("targetPoseAngle", m_targetPose.getRotation().getRadians());

    }

    // returns estimated position based on odometry
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    // returns the heading of the robot
    public Rotation2d getHeading() {
        return m_gyro.getRotation2d();
    }

    public void brake() {
        for (SwerveModule module : m_swerveModules) {
            module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
        }
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        // SwerveModule.normalizeWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        setModuleStates(moduleStates);

    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < 4; i++) {
            m_swerveModules[i].setDesiredState(moduleStates[i]);
        }
    }

    public void setDesiredTurn(SwerveModuleState state) {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = state;
        }
        setModuleStates(moduleStates);
    }

    public Rotation2d getGyroAngle() {
        return m_gyro.getRotation2d();
    }
}
