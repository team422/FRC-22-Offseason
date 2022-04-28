package frc.robot.commands.climbcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class ClimberPIDPos extends CommandBase {
    private final Climber climber;

    public ClimberPIDPos(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void execute() {
        climber.SetTarget(4086);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
