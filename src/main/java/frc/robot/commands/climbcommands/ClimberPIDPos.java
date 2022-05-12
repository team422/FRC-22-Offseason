package frc.robot.commands.climbcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class ClimberPIDPos extends CommandBase {
    private final Climber climber;
    private final double position;

    public ClimberPIDPos(Climber climber, double position) {
        this.climber = climber;
        this.position = position;
    }

    @Override
    public void execute() {
        climber.SetTarget(position);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
