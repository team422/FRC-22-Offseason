package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    SwerveDrivePoseEstimator m_odometry;
    // Swerve Drive Odometry with vision correction

    String m_swerveModuleNames[] = { "Left Front", "Right Front", "Left Rear", "Right Rear" };

    //target pose and controller
    Pose2d m_targetPose;
    PIDController m_thetaController = new PIDController(1.0, 0.0, 0.05);
    int m_currentWheel = 0;
    Boolean m_singleWheelMode = false;

    public FullSwerveBase(SwerveModule[] m_swerveModules, Gyro gyro) {
        // Setting up all the modules

        this.m_swerveModules = m_swerveModules;
        for (SwerveModule module : m_swerveModules) {
            module.resetDistance();
            // module.syncTurningEncoders();
            // module.DONTUSETHISRESETTURNINGENCODER();
        }
        // m_targetPose = m_odometry.getPoseMeters();
        m_thetaController.reset();
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.m_gyro = gyro;
        m_gyro.calibrate();
        m_gyro.reset();

        // odometry stuff
        // SwerveModulePositions[] modulePositions = new SwerveModulePositions[4];
        // double[][] stdDeviations = new double[3][1];
        // Matrix<N3, N1> stdDeviations = new Matrix(new Nat<N3>(3), new Nat<N1>(1));

        // stdDeviations.set(0, 0, 0.1);
        // stdDeviations.set(1, 0, 0.1);
        // stdDeviations.set(2, 0, 0.1);
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(m_swerveModules[i].getDriveDistanceMeters(),
                    m_swerveModules[i].getTurnDegrees());
        }
        // SwerveModulePositions[] modulePositions = new SwerveModulePositions[4];
        // Matrix<Integer, Double> stdDevMatrix = new Matrix<>(3, 4);
        // Rotation2d gyroAngle,
        //   Pose2d initialPoseMeters,
        //   SwerveDriveKinematics kinematics,
        //   Matrix<N3, N1> stateStdDevs,
        //   Matrix<N1, N1> localMeasurementStdDevs,
        //   Matrix<N3, N1> visionMeasurementStdDevs)
        // m_odometry = new SwerveDrivePoseEstimator(this.getHeading(), new Pose2d(), DriveConstants.kDriveKinematics,stdDeviations, stdDeviations, stdDeviations);
        m_odometry = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, this.getHeading(), modulePositions,
                new Pose2d());
        // // DriveConstants.kDriveKinematics,  
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /* 
        m_odometry.update(this.getHeading(), m_swerveModules[0].getState(), m_swerveModules[1].getState(),
                m_swerveModules[2].getState(), m_swerveModules[3].getState());
         */
        //This was literally all copy paste, these should be good for debugging
        // SmartDashboard.putNumber("Heading", getHeading().getDegrees());
        // SmartDashboard.putNumber("currentX", getPose().getX());
        // SmartDashboard.putNumber("currentY", getPose().getY());
        // SmartDashboard.putNumber("currentAngle", getPose().getRotation().getRadians());

        SmartDashboard.putNumber("Left Front Absolute", m_swerveModules[0].getAbsoluteRotation().getDegrees() % 360);
        SmartDashboard.putNumber("Right Front Absolute", m_swerveModules[1].getAbsoluteRotation().getDegrees() % 360);
        SmartDashboard.putNumber("Left Rear Absolute", m_swerveModules[2].getAbsoluteRotation().getDegrees() % 360);
        SmartDashboard.putNumber("Right Rear Absolute", m_swerveModules[3].getAbsoluteRotation().getDegrees() % 360);
        SmartDashboard.putNumber("Left Front encoder", m_swerveModules[0].getTurnDegrees().getDegrees());
        SmartDashboard.putNumber("Right Front encoder", m_swerveModules[1].getTurnDegrees().getDegrees());
        SmartDashboard.putNumber("Left Rear encoder", m_swerveModules[2].getTurnDegrees().getDegrees());
        SmartDashboard.putNumber("Right Rear encoder", m_swerveModules[3].getTurnDegrees().getDegrees());

        // SmartDashboard.putNumber("targetPoseAngle", m_targetPose.getRotation().getRadians());
        /*
        for (int i = 0; i < m_swerveModules.length; i++) {
            SmartDashboard.putNumber(m_swerveModuleNames[i] + ": Drive Speed",
                    m_swerveModules[i].getDriveVelocityMetersPerSecond());
            SmartDashboard.putNumber(m_swerveModuleNames[i] + ": Rotation",
                    m_swerveModules[i].getTurnDegrees());
        }
         */
    }

    // returns estimated position based on odometry
    public Pose2d getPose() {
        // return new Pose2d();
        // return m_odometry.getPoseMeters();
        return new Pose2d();
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
                    m_swerveModules[i].getTurnDegrees());
        }
        if (!this.m_singleWheelMode) {
            // SwerveModule.normalizeWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);
            setModuleStates(moduleStatesFinal);
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

    public void addVisionOdometry(Pose2d pose, double timestamp) {

    }
}
