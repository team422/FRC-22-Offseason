package frc.robot.commands.climbcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

public class ClimberPIDPos extends CommandBase {
    private final Climber climber;
    private final double position;
    private final boolean approximationMode;
    private boolean isAtSetPoint;
    private int counter;
    private double settleTime = 1 / Constants.loopPeriodSecs; //gives settle time in ticks by changing the numerator.

    public ClimberPIDPos(Climber climber, double position, boolean approximationMode) {
        this.climber = climber;
        this.position = position;
        this.approximationMode = approximationMode;
    }

    @Override
    public void execute() {
        climber.SetTarget(position);
        isAtSetPoint = climber.isAtSetPoint();
        if (isAtSetPoint) {
            counter++;
        } else {
            counter = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.setPercent(0);
    }

    @Override
    public boolean isFinished() {
        if (approximationMode) {
            return counter >= settleTime;
        } else {
            return false;
        }
    }
}
