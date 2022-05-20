package frc.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.Climber;
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
    private final boolean isTuningPosition;

    public PIDTuning(boolean isTuningMode, boolean isTuningPosition) {
        this.isTuningMode = isTuningMode;
        this.isTuningPosition = isTuningPosition;
        if (isTuningMode) {
            climber = new Climber(new ClimberIOFalcon(), new ClimberPistonIO());
            //shuffle board magik documentation below
            shuffleBoardMagik();
            //add listeners to the tunable numbers, these are consumers that will take in an input and perform an action using said input
            if (isTuningPosition) {
                climber.setClimberPositionPID(f, p, i, d);
                climber.configurePositionPID();
                TunableF.addChangeListener((val) -> climber.setClimberPositionPID(val, p, i, d));
                TunableP.addChangeListener((val) -> climber.setClimberPositionPID(f, val, i, d));
                TunableI.addChangeListener((val) -> climber.setClimberPositionPID(f, p, val, d));
                TunableD.addChangeListener((val) -> climber.setClimberPositionPID(f, p, i, val));
                TunableSetPoint.addChangeListener((val) -> climber.SetTarget(val)); //ex: takes in some double which is accepted when the value of the tunnable number changes and uses it to set the target set point.
            } else {
                climber.setClimberVelocityPID(f, p, i, d);
                climber.configureVelocityPID();
                TunableF.addChangeListener((val) -> climber.setClimberVelocityPID(val, p, i, d));
                TunableP.addChangeListener((val) -> climber.setClimberVelocityPID(f, val, i, d));
                TunableI.addChangeListener((val) -> climber.setClimberVelocityPID(f, p, val, d));
                TunableD.addChangeListener((val) -> climber.setClimberVelocityPID(f, p, i, val));
                TunableSetPoint.addChangeListener((val) -> climber.setVelocity(val));
            }
        }
    }

    //initializing and updating shuffle board (pretty graphs yay)
    private void shuffleBoardMagik() {
        SmartDashboard.putNumber("Encoder Position", climber.getEncoderPosition());
        SmartDashboard.putNumber("Error", climber.getEncoderPosition() - setpoint); //gives how much overshoot/undershoot you have positive number means overshoot and negative means undershoot
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
            System.out.println(isTuningPosition ? "Position controller" : "Velocity Controller");
            System.out.println("F Gain" + f);
            System.out.println("P Gain" + p);
            System.out.println("I Gain" + i);
            System.out.println("D Gain" + d);
            System.out.println("Enjoy ;)");
        }
    }
}
