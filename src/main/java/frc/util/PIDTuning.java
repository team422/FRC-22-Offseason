package frc.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.Climber;

public class PIDTuning {
    private Climber climber;

    private double kf = 0;
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double setPoint = 0;

    public PIDTuning(boolean isTuningMode) {
        if (isTuningMode) {
            shuffleBoardMagik();
        }
    }

    private void shuffleBoardMagik() {
        SmartDashboard.putNumber("P Gain", kp);
        SmartDashboard.putNumber("I Gain", ki);
        SmartDashboard.putNumber("D Gain", kd);
    }

    public void Tune() {
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

        if ((p != kp) || (i != ki) || (d != kd) || (f != kf)) {
            climber.setClimberPID(f, p, i, d);
            kf = f;
            kp = p;
            ki = i;
            kd = d;
        }

        if (setpoint != setPoint) {
            climber.SetTarget(setpoint);
            setPoint = setpoint;
        }
        SmartDashboard.putNumber("Encoder Position", climber.getEncoderPosition());
        SmartDashboard.putNumber("Error", climber.getEncoderPosition() - setpoint);
    }

    public void getTunedValues() {
        System.out.println("F Gain" + kf);
        System.out.println("P Gain" + kp);
        System.out.println("I Gain" + ki);
        System.out.println("D Gain" + kd);
        System.out.println("Enjoy ;)");
    }
}
