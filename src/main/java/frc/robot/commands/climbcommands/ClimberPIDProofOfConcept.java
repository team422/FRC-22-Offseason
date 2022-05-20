package frc.robot.commands.climbcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.climber.Climber;

public class ClimberPIDProofOfConcept extends ParallelCommandGroup {
    public ClimberPIDProofOfConcept(Climber climber) {
        addCommands(
                sequence(new ClimberPIDPos(climber, 1000.0, true), new ClimberPIDPos(climber, 2000.0, false)));
    }
}
