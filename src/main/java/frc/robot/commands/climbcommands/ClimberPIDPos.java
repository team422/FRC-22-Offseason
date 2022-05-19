package frc.robot.commands.climbcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class ClimberPIDPos extends CommandBase {
    private final Climber climber;
    private final double position;
    private final boolean approximationMode;
    private boolean isAtSetPoint;

    public ClimberPIDPos(Climber climber, double position, boolean approximationMode) {
        this.climber = climber;
        this.position = position;
        this.approximationMode = approximationMode;
    }

    @Override
    public void execute() {
        climber.SetTarget(position);
        isAtSetPoint = climber.isAtSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        climber.setPercent(0);
    }

    @Override
    public boolean isFinished() {
        if (approximationMode) {
            return isAtSetPoint;
        } else {
            return false;
        }
    }
}
