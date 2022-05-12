package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private double kP = 0, kI = 0, kD = 0, kF = 0;
    private final ClimberIO climberIO;
    private final ClimberPistonIO pistonIO;

    public Climber(ClimberIO climberIO, ClimberPistonIO pistonIO) {
        this.climberIO = climberIO;
        this.pistonIO = pistonIO;

        setClimberPID(kF, kP, kI, kD);
        setBrakeMode(true);
    }

    public void setClimberPID(double F, double P, double I, double D) {
        climberIO.setPID(F, P, I, D);
    }

    public void setBrakeMode(boolean enable) {
        climberIO.setBrakeMode(enable);
    }

    public void setVelocity(double velocity) {
        climberIO.setVelocity(velocity);
    }

    // public void setRightVelocity(double velocity){
    //     leftClimberIO.setVelocity(velocity);
    // }

    public void setPercent(double percent) {
        climberIO.setPercentPower(percent);
    }

    public void setRightPercent(double percent) {
        climberIO.setRightPercent(percent);
    }

    public void setLeftPercent(double percent) {
        climberIO.setLeftPercent(percent);
    }

    public void SetTarget(double encoderValue) {
        climberIO.setTargetPoint(encoderValue);
    }

    // public void setRightTarget(double encoderValue) {
    //     rightClimberIO.setTargetPoint(encoderValue);
    // }

    public void tiltRobot() {
        if (pistonIO.getTilt()) {
            pistonIO.tiltRobot(false);
        } else {
            pistonIO.tiltRobot(true);
        }
    }

    public double getEncoderPosition() {
        return climberIO.getEncoderPosition();
    }
}
