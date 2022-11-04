package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;

public class Turn extends CommandBase {
    public SwerveModule m_swerveModule;
    public double angle;

    public Turn(SwerveModule module, double angle) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_swerveModule = module;
        this.angle = angle;

    }

    @Override
    public void execute() {
        m_swerveModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(angle)));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_swerveModule.getState().angle.getDegrees() - angle) > 2) {
            return true;
        }
        return false;
    }
}
