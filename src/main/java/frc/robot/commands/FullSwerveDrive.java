package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.FullSwerveBase;

public class FullSwerveDrive extends CommandBase {

    Supplier<Double> xSpeed;
    Supplier<Double> ySpeed;
    Supplier<Double> zRotation;
    double curXSpeed;
    double curYSpeed;
    double curZRotation;
    FullSwerveBase swerveBase;
    ChassisSpeeds speeds;

    public FullSwerveDrive(FullSwerveBase swerveDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
            Supplier<Double> zRotation) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
        this.swerveBase = swerveDrive;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zRotation = zRotation;

    }

    public void initialize() {
        // swerveBase.drive(xSpeed.get(), ySpeed.get(), zRotation.get());
    }

    public void execute() {
        curXSpeed = xSpeed.get() * DriveConstants.kMaxSpeedMetersPerSecond;
        curYSpeed = ySpeed.get() * DriveConstants.kMaxSpeedMetersPerSecond;
        curZRotation = zRotation.get() * DriveConstants.kMaxAngularSpeed;

        speeds = new ChassisSpeeds(curXSpeed, curYSpeed, curZRotation);
        swerveBase.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveBase.brake();
    }

}
