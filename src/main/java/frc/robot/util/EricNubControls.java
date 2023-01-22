package frc.robot.util;

public class EricNubControls {
    public EricNubControls() {
    }

    public double addDeadzoneBasic(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        } else {
            return input;
        }
    }

    public double addDeadzoneScaled(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        } else {
            return (input - Math.signum(input) * deadzone) / (1 - deadzone);
        }
    }

    public double addEricCurve(double input) {
        if (input < 0.8) {
            return (1 / 3) * input + (1 / 9) * input * input + (65 / 72) * input * input * input;
        } else {
            return (1 / 5) * Math.pow(5 * input - 5, 3) + 1;
        }
    }
}
