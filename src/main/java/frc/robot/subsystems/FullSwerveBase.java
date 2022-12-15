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
     * Creates a new this.
     */
    // these should be in the order front left, front right, back left, back right
    SwerveModule m_swerveModules[] = new SwerveModule[4];
    Gyro m_gyro;
    SwerveDriveOdometry m_odometry;

    String m_swerveModuleNames[] = { "Left Front", "Right Front", "Left Rear", "Right Rear" };

    //target pose and controller
    Pose2d m_targetPose;
    PIDController m_thetaController = new PIDController(1.0, 0.0, 0.05);
    int m_currentWheel = 0;
    Boolean m_singleWheelMode = true;

    public FullSwerveBase(SwerveModule[] m_swerveModules, ADXRS450_Gyro gyro) {
        // Setting up all the modules
        this.m_swerveModules = m_swerveModules;
        for (SwerveModule module : m_swerveModules) {
            module.resetDistance();
            module.syncTurningEncoders();
            // module.DONTUSETHISRESETTURNINGENCODER();
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

        for (int i = 0; i < m_swerveModules.length; i++) {
            SmartDashboard.putNumber(m_swerveModuleNames[i] + ": Drive Speed",
                    m_swerveModules[i].getDriveVelocityMetersPerSecond());
            SmartDashboard.putNumber(m_swerveModuleNames[i] + ": Rotation",
                    m_swerveModules[i].getTurnDegrees());
        }

    }

    // returns estimated position based on odometry
    public Pose2d getPose() {
        // return new Pose2d();
        return m_odometry.getPoseMeters();
    }

    public void printAllVals() {
        // for (SwerveModule iModule : this.m_swerveModules) {
        //     System.out.println(""+iModule.getState());
        // }
        for (int i = 0; i < 4; i++) {
            System.out.println(m_swerveModuleNames[i] + m_swerveModules[i].getState());
        }
    }

    public void switchWheel() {
        if (m_currentWheel == 3) {
            m_currentWheel -= 4;
        }
        m_currentWheel = m_currentWheel + 1;

    }

    public void switchTestingMode() {
        m_singleWheelMode = !m_singleWheelMode;
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
        // SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        // SwerveModule.normalizeWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveModuleState[] moduleStatesFinal = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStatesFinal[i] = SwerveModuleState.optimize(moduleStates[i],
                    new Rotation2d(m_swerveModules[i].getTurnDegrees()));
        }
        if (!this.m_singleWheelMode) {
            // SwerveModule.normalizeWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);
            setModuleStates(moduleStates);
        } else {
            m_swerveModules[this.m_currentWheel]
                    .setDesiredState(moduleStates[this.m_currentWheel]);
            System.out.println("Wanted:" + moduleStates[this.m_currentWheel]);
            System.out.println("Actual Degree:" + m_swerveModules[this.m_currentWheel].getTurnDegrees());
            // System.out.println("Actual" + m_swerveModules[this.m_currentWheel].getState());
        }

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
