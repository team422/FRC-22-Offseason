package frc.util;

import java.util.Random;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.Climber;

public class PIDTuning {
    private Climber climber;

    TunableNumber TunableP = new TunableNumber("P Gain", 0);
    TunableNumber TunableI = new TunableNumber("I Gain", 0);
    TunableNumber TunableD = new TunableNumber("D Gain", 0);
    TunableNumber TunableF = new TunableNumber("F Gain", 0);
    TunableNumber TunableSetPoint = new TunableNumber("Set Point", 0);
    Random rnjesus = new Random();
    double AnkitsFait = 0;
    double setpoint = TunableSetPoint.get();
    double p = TunableP.get();
    double i = TunableI.get();
    double d = TunableD.get();
    double f = TunableF.get();

    private final boolean isTuningMode;

    public PIDTuning(boolean isTuningMode) {
        this.isTuningMode = isTuningMode;
        if (isTuningMode) {
            shuffleBoardMagik();
            TunableF.addChangeListener((val) -> climber.setClimberPID(val, p, i, d));
            TunableP.addChangeListener((val) -> climber.setClimberPID(f, val, i, d));
            TunableI.addChangeListener((val) -> climber.setClimberPID(f, p, val, d));
            TunableD.addChangeListener((val) -> climber.setClimberPID(f, p, i, val));
            TunableSetPoint.addChangeListener((val) -> climber.SetTarget(val));
        }
    }

    private void shuffleBoardMagik() {
        SmartDashboard.putNumber("P Gain", p);
        SmartDashboard.putNumber("I Gain", i);
        SmartDashboard.putNumber("D Gain", d);
        SmartDashboard.putNumber("Encoder Position", climber.getEncoderPosition());
        SmartDashboard.putNumber("Error", climber.getEncoderPosition() - setpoint);
        SmartDashboard.putNumber("Set Point", setpoint);
    }

    public void Tune() {
        if (isTuningMode) {
            boolean AnkitMode = SmartDashboard.getBoolean("Ankit Mode", false);
            shuffleBoardMagik();

            p = TunableP.get();
            i = TunableI.get();
            d = TunableD.get();
            f = TunableF.get();
            if (AnkitMode) {
                int counter = 0;
                if (counter >= 100) {
                    AnkitsFait = rnjesus.nextInt(8192);
                    counter = 0;
                    setpoint = AnkitsFait;
                } else {
                    counter++;
                }
            } else {
                setpoint = TunableSetPoint.get();
            }
        }
    }

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
