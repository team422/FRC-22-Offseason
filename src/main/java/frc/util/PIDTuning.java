package frc.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.climber.ClimberPistonIO;

public class PIDTuning {

    private Climber climber;

    //defines the tunnable numbers necessary for this to work
    TunableNumber TunableP = new TunableNumber("P Gain", 0);
    TunableNumber TunableI = new TunableNumber("I Gain", 0);
    TunableNumber TunableD = new TunableNumber("D Gain", 0);
    TunableNumber TunableF = new TunableNumber("F Gain", 0);
    TunableNumber TunableSetPoint = new TunableNumber("Set Point", 0);

    //variables that store the tunnable numbers in a usable form so they can be used later or printed later
    double setpoint = TunableSetPoint.get();
    double p = TunableP.get();
    double i = TunableI.get();
    double d = TunableD.get();
    double f = TunableF.get();

    //tuning mode boolean If this is false then tunning will not take place, the Tunable numbers will all return default values
    private final boolean isTuningMode;

    public PIDTuning(boolean isTuningMode) {
        this.isTuningMode = isTuningMode;
        if (isTuningMode) {
            climber = new Climber(new ClimberIOFalcon(), new ClimberPistonIO());
            //shuffle board magik documentation below
            shuffleBoardMagik();
            //add listeners to the tunable numbers, these are consumers that will take in an input and perform an action using said input
            TunableF.addChangeListener((val) -> climber.setClimberPositionPID(val, p, i, d));
            TunableP.addChangeListener((val) -> climber.setClimberPositionPID(f, val, i, d));
            TunableI.addChangeListener((val) -> climber.setClimberPositionPID(f, p, val, d));
            TunableD.addChangeListener((val) -> climber.setClimberPositionPID(f, p, i, val));
            TunableSetPoint.addChangeListener((val) -> climber.SetTarget(val, p, i, d, f)); //ex: takes in some double which is accepted when the value of the tunnable number changes and uses it to set the target set point.
        } else {
            climber = climber != null ? climber : new Climber(new ClimberIO() {
            }, null);
        }
    }

    //initializing and updating shuffle board (pretty graphs yay)
    private void shuffleBoardMagik() {
        SmartDashboard.putNumber("P Gain", p);
        SmartDashboard.putNumber("I Gain", i);
        SmartDashboard.putNumber("D Gain", d);
        SmartDashboard.putNumber("Encoder Position", climber.getEncoderPosition());
        SmartDashboard.putNumber("Error", climber.getEncoderPosition() - setpoint); //gives how much overshoot/undershoot you have positive number means overshoot and negative means undershoot
        SmartDashboard.putNumber("Set Point", setpoint);
    }

    //method that runs periodically to update things (put in teleopperiodic)
    public void Tune() {
        if (isTuningMode) {
            //updates shuffle board
            shuffleBoardMagik();

            //update useable numbers as well as check to see if any changes so you can notify comsumers
            p = TunableP.get();
            i = TunableI.get();
            d = TunableD.get();
            f = TunableF.get();
        }
    }

    //when tuning is terminated, print out the values you had so you don't lose your progress
    public void getTunedValues() {
        if (isTuningMode) {
            System.out.println("F Gain" + p);
            System.out.println("P Gain" + i);
            System.out.println("I Gain" + d);
            System.out.println("D Gain" + f);
            System.out.println("Enjoy ;)");
        }
    }
}
