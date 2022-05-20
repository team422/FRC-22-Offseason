package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private double kPPos = 0, kIPos = 0, kDPos = 0, kFPos = 0;
    private double kPVel = 0, kIVel = 0, kDVel = 0, kFVel = 0;
    private final ClimberIO climberIO;
    private final ClimberPistonIO pistonIO;

    public Climber(ClimberIO climberIO, ClimberPistonIO pistonIO) {
        this.climberIO = climberIO;
        this.pistonIO = pistonIO;

        setClimberPositionPID(kFPos, kPPos, kIPos, kDPos);
        setClimberVelocityPID(kFVel, kPVel, kIVel, kDVel);
        setBrakeMode(true);
    }

    public boolean isAtSetPoint() {
        return climberIO.isAtSetPoint();
    }

    public double getCurrentSetPoint() {
        return climberIO.getSetPoint();
    }

    public void setClimberPositionPID(double F, double P, double I, double D) {
        climberIO.setPID(F, P, I, D, true);
    }

    public void configurePositionPID() {
        climberIO.configurePositionPID();
    }

    public void configureVelocityPID() {
        climberIO.configureVelocityPID();
    }

    public void setClimberVelocityPID(double F, double P, double I, double D) {
        climberIO.setPID(F, P, I, D, false);
    }

    public void setBrakeMode(boolean enable) {
        climberIO.setBrakeMode(enable);
    }

    public void setVelocity(double velocity) {
        climberIO.configureVelocityPID();
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
        climberIO.configurePositionPID();
        climberIO.setTargetPoint(encoderValue);
    }

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

    /**gets what mode the PID controller is currently in*/
    public void getClimberPIDMode() {
        System.out.println("Current PID loops is in" +
                (climberIO.isInPosPIDMode() ? "Position Mode" : "Velocity Mode"));
    }
}
