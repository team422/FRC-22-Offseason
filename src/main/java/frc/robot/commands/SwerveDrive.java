package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.mechtechsupport.util.SwerveModuleMath;
import frc.robot.subsystems.drive.DriveBase;

public class SwerveDrive extends CommandBase {
    private final DriveBase drive;
    private final Supplier<Double> Yspeed;
    private final Supplier<Double> Xspeed;
    private final Supplier<Double> rot;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveDrive(DriveBase drivetrain, Supplier<Double> Yspeed,
            Supplier<Double> Xspeed, Supplier<Double> rot) {
        this.drive = drivetrain;
        this.Yspeed = Yspeed;
        this.Xspeed = Xspeed;
        this.rot = rot;

        this.xLimiter = new SlewRateLimiter(4); //all of these are guesses change base on matthew prefference
        this.yLimiter = new SlewRateLimiter(4);
        this.turningLimiter = new SlewRateLimiter(Math.PI * 2);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        runSwerve();
    }

    private void runSwerve() {
        double xSpeed = Xspeed.get();
        double ySpeed = Yspeed.get();
        double turningSpeed = rot.get() * (0.5);

        xSpeed = Math.abs(xSpeed) > 0.01 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.01 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > 0.01 ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * 5;
        ySpeed = yLimiter.calculate(ySpeed) * 5;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Math.PI * 2;

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed,
                Rotation2d.fromDegrees(-drive.getHeading()));

        drive.setStates(SwerveModuleMath.swerveMagik.toSwerveModuleStates(chassisSpeeds));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
