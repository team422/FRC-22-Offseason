package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.util.SwerveModuleConfig;
import frc.util.SwerveModuleMath;
import frc.util.SwerveOdometer;

public class SwerveDriveIO implements DriveIO {

    private SwerveModuleConfig frontRight;
    private SwerveModuleConfig frontLeft;
    private SwerveModuleConfig backRight;
    private SwerveModuleConfig backLeft;
    private ADXRS450_Gyro gyro;
    private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

    public static double kPSteerFR = 0;
    public static double kISteerFR = 0;
    public static double kDSteerFR = 0;
    public static double kFSteerFR = 0;
    public static double kPDriveFR = 0;
    public static double kIDriveFR = 0;
    public static double kDDriveFR = 0;
    public static double kFDriveFR = 0;

    public static double kPSteerFL = 0;
    public static double kISteerFL = 0;
    public static double kDSteerFL = 0;
    public static double kFSteerFL = 0;
    public static double kPDriveFL = 0;
    public static double kIDriveFL = 0;
    public static double kDDriveFL = 0;
    public static double kFDriveFL = 0;

    public static double kPSteerBR = 0;
    public static double kISteerBR = 0;
    public static double kDSteerBR = 0;
    public static double kFSteerBR = 0;
    public static double kPDriveBR = 0;
    public static double kIDriveBR = 0;
    public static double kDDriveBR = 0;
    public static double kFDriveBR = 0;

    public static double kPSteerBL = 0;
    public static double kISteerBL = 0;
    public static double kDSteerBL = 0;
    public static double kFSteerBL = 0;
    public static double kPDriveBL = 0;
    public static double kIDriveBL = 0;
    public static double kDDriveBL = 0;
    public static double kFDriveBL = 0;
    private static SwerveOdometer odometer;

    private SwerveModuleConfig[] moduleContainer;

    public SwerveDriveIO() {
        switch (Constants.bot) {
            case SWERVEPROTOTYPE:
                odometer = new SwerveOdometer();
                frontRight = new SwerveModuleConfig(1, 5, 45, kPSteerFR, kISteerFR, kDSteerFR, kFSteerFR, kPDriveFR,
                        kIDriveFR, kDDriveFR, kFDriveFR);
                frontLeft = new SwerveModuleConfig(2, 6, -45, kPSteerFL, kISteerFL, kDSteerFL, kFSteerFL, kPDriveFL,
                        kIDriveFL, kDDriveFL, kFDriveFL);
                backRight = new SwerveModuleConfig(3, 7, 135, kPSteerBR, kISteerBR, kDSteerBR, kFSteerBR, kPDriveBR,
                        kIDriveBR, kDDriveBR, kFDriveBR);
                backLeft = new SwerveModuleConfig(4, 8, -135, kPSteerBL, kISteerBL, kDSteerBL, kFSteerBL, kPDriveBL,
                        kIDriveBL, kDDriveBL, kFDriveBL);
                moduleContainer = new SwerveModuleConfig[] { frontRight, frontLeft, backRight, backLeft };
                break;
            default:
                throw new RuntimeException("Invalid robot for Swerve");
        }
    }

    @Override
    public void setSteerPID(double[] Pcontainer, double[] Icontainer, double[] Dcontainer, double[] Fcontainer) {
        for (int i = 0; i < moduleContainer.length - 1; i++) {
            moduleContainer[i].configSteerPID(Pcontainer[i], Icontainer[i], Dcontainer[i], Fcontainer[i]);
        }
    }

    @Override
    public void setDrivePID(double[] Pcontainer, double[] Icontainer, double[] Dcontainer, double[] Fcontainer) {
        for (int i = 0; i < moduleContainer.length - 1; i++) {
            moduleContainer[i].configDrivePID(Pcontainer[i], Icontainer[i], Dcontainer[i], Fcontainer[i]);
        }
    }

    @Override
    public double getGyroAngle() {
        return gyro.getAngle();
    }

    @Override
    public void resetGyroAngle() {
        gyro.reset();
    }

    @Override
    public void calibrateGyro() {
        gyro.calibrate();
    }

    @Override
    public double getGyroRate() {
        return gyro.getRate();
    }

    @Override
    public double getHeading() {
        return SwerveModuleMath.boundPM180(gyro.getAngle());
    }

    @Override
    public void updateOdometry() {
        //Speeds of the wheels in meters per second
        double speedFR = frontRight.getDriveVelocity();
        double speedFL = frontLeft.getDriveVelocity();
        double speedBR = backRight.getDriveVelocity();
        double soeedBL = backLeft.getDriveVelocity();
        //Angles of the wheels (0, 360)
        double heading = getHeading();
        double angleFR = frontRight.getAngle() + heading;
        double angleFL = frontLeft.getAngle() + heading;
        double angleBR = backRight.getAngle() + heading;
        double angleBL = backLeft.getAngle() + heading;
        //The vector components of the wheels, based on their current values
        double FRX = SwerveModuleMath.getXHeading(angleFR) * speedFR;
        double FRY = SwerveModuleMath.getYHeading(angleFR) * speedFR;
        double FLX = SwerveModuleMath.getXHeading(angleFL) * speedFL;
        double FLY = SwerveModuleMath.getYHeading(angleFL) * speedFL;
        double BRX = SwerveModuleMath.getXHeading(angleBR) * speedBR;
        double BRY = SwerveModuleMath.getYHeading(angleBR) * speedBR;
        double BLX = SwerveModuleMath.getXHeading(angleBL) * soeedBL;
        double BLY = SwerveModuleMath.getYHeading(angleBL) * soeedBL;

        //vector magik
        double OdometricX = (FRX + FLX + BRX + BLX) / 4D;
        double OdometricY = (FRY + FLY + BRY + BLY) / 4D;

        double deltaY = OdometricX * Constants.loopPeriodSecs;
        double deltaX = OdometricY * Constants.loopPeriodSecs;
        odometer.update(deltaX, deltaY);
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(odometer.getPoseMeters(), Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void stap() {
        frontRight.stop();
        frontLeft.stop();
        backRight.stop();
        backLeft.stop();
    }

    @Override
    public void setStates(SwerveModuleState[] stateContainer) {
        for (int i = 0; i < moduleContainer.length - 1; i++) {
            moduleContainer[i].setDesiredState(stateContainer[i]);
        }
    }
}
