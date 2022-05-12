package frc.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.Climber;

public class PIDTuning {
    private Climber climber;

    TunableNumber TunableP = new TunableNumber("P Gain", 0);
    TunableNumber TunableI = new TunableNumber("I Gain", 0);
    TunableNumber TunableD = new TunableNumber("D Gain", 0);
    TunableNumber TunableF = new TunableNumber("F Gain", 0);
    TunableNumber TunableSetPoint = new TunableNumber("Set Point", 0);
    double setpoint = TunableSetPoint.get();
    double p = TunableP.get();
    double i = TunableI.get();
    double d = TunableD.get();
    double f = TunableF.get();

    double currentSetPoint;

    public PIDTuning(boolean isTuningMode) {
        if (isTuningMode) {
            shuffleBoardMagik();
            TunableF.addChangeListener((val) -> climber.setClimberPID(val, p, i, d));
            TunableP.addChangeListener((val) -> climber.setClimberPID(f, val, i, d));
            TunableI.addChangeListener((val) -> climber.setClimberPID(f, p, val, d));
            TunableD.addChangeListener((val) -> climber.setClimberPID(f, p, i, val));
            TunableSetPoint.addChangeListener((val) -> climber.SetTarget(val));
            TunableSetPoint.addChangeListener((val) -> currentSetPoint = val);
        }
    }

    private void shuffleBoardMagik() {
        SmartDashboard.putNumber("P Gain", p);
        SmartDashboard.putNumber("I Gain", i);
        SmartDashboard.putNumber("D Gain", d);
    }

    public void Tune(boolean isTuningMode) {
        if (isTuningMode) {
            // if ((p != kp) || (i != ki) || (d != kd) || (f != kf)) {
            //     climber.setClimberPID(f, p, i, d);
            //     kf = f;
            //     kp = p;
            //     ki = i;
            //     kd = d;
            // }

            // if (setpoint != setPoint) {
            //     climber.SetTarget(setpoint);
            //     setPoint = setpoint;
            // }

            p = TunableP.get();
            i = TunableP.get();
            d = TunableP.get();
            f = TunableP.get();

            SmartDashboard.putNumber("Encoder Position", climber.getEncoderPosition());
            SmartDashboard.putNumber("Error", climber.getEncoderPosition() - currentSetPoint);
        }
    }

    public void getTunedValues(boolean isTuningMode) {
        if (isTuningMode) {
            System.out.println("F Gain" + p);
            System.out.println("P Gain" + i);
            System.out.println("I Gain" + d);
            System.out.println("D Gain" + f);
            System.out.println("Enjoy ;)");
        }
    }
}
