package frc.util;

import java.util.Random;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.Climber;

public class PIDTuning {
    private Climber climber;

    //defines the tunnable numbers necessary for this to work
    TunableNumber TunableP = new TunableNumber("P Gain", 0);
    TunableNumber TunableI = new TunableNumber("I Gain", 0);
    TunableNumber TunableD = new TunableNumber("D Gain", 0);
    TunableNumber TunableF = new TunableNumber("F Gain", 0);
    TunableNumber TunableSetPoint = new TunableNumber("Set Point", 0);
    //meme things that may actually be applicable in testing. (rapid stress testing of PID loop)
    Random AnkitsFait = new Random(); //random number generator
    int counter = 0; //counter value, counts up every tick (0.02 sec) when a condition is or isn't met. Once it reaches a point then it will cause an if statement to fire
    /**
     * Story time about Ankits fait. There was once a build subteam lead named Ankit. When he was building the
     * climb arms, he let go. The climb arms then flew into his forehead and caused him to bleed a little. 
     * From then on out Ankit was immortalized as the man who was hit by a spring loaded climb arm.
     * As tribute, this random number generator was named in his honor as it causes the climb arms or whatever
     * you are tuning to randomly go to a position which may bonk someone's head.
     */

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
            //shuffle board magik documentation below
            shuffleBoardMagik();
            //add listeners to the tunable numbers, these are consumers that will take in an input and perform an action using said input
            TunableF.addChangeListener((val) -> climber.setClimberPID(val, p, i, d));
            TunableP.addChangeListener((val) -> climber.setClimberPID(f, val, i, d));
            TunableI.addChangeListener((val) -> climber.setClimberPID(f, p, val, d));
            TunableD.addChangeListener((val) -> climber.setClimberPID(f, p, i, val));
            TunableSetPoint.addChangeListener((val) -> climber.SetTarget(val)); //ex: takes in some double which is accepted when the value of the tunnable number changes and uses it to set the target set point.
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
            //checks if ankit mode (random mode to stress test)
            boolean AnkitMode = SmartDashboard.getBoolean("Ankit Mode", false);
            //updates shuffle board
            shuffleBoardMagik();

            //update useable numbers as well as check to see if any changes so you can notify comsumers
            p = TunableP.get();
            i = TunableI.get();
            d = TunableD.get();
            f = TunableF.get();

            //memes, if Ankit mode is enabled, then it will produce a random int ever 100 ticks and set the motor to go to that point (can be used to test the viability of a PID loop)
            if (AnkitMode) {
                if (counter >= 100) {
                    counter = 0;
                    setpoint = AnkitsFait.nextInt(TalonFXUtils.TICKS_PER_REVOLUTION * 4); //warning some mechanisms may not have enough room for 4 falcon 500 rotations, adjust accordingly
                } else {
                    counter++;
                }
            } else {
                setpoint = TunableSetPoint.get();
            }
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
