package frc.mechtechsupport.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mechtechsupport.modules.SwerveModuleConfig;
import frc.robot.Constants;

public class PIDTuning {
    //TODO: make PID Tuner for swerve
    private static double errorVel;
    private static double setPointVel;
    private static double errorPos;
    private static double setPointPos;
    private static SwerveModuleConfig module;
    private static double pSteer = 0;
    private static double iSteer = 0;
    private static double dSteer = 0;
    private static double fSteer = 0;
    private static double pDrive = 0;
    private static double iDrive = 0;
    private static double dDrive = 0;
    private static double fDrive = 0;
    private static int driveID = 0;
    private static int steerID = 0;
    private static TunableNumber tunableSteerP;
    private static TunableNumber tunableSteerI;
    private static TunableNumber tunableSteerD;
    private static TunableNumber tunableSteerF;
    private static TunableInt tunableSteerID;
    private static TunableNumber tunableHeading;
    private static TunableNumber tunableDriveP;
    private static TunableNumber tunableDriveI;
    private static TunableNumber tunableDriveD;
    private static TunableNumber tunableDriveF;
    private static TunableInt tunableDriveID;
    private static TunableNumber tunableSpeed;

    public PIDTuning() {
        if (Constants.tuningMode) {
            module = new SwerveModuleConfig(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

            errorVel = 0;
            setPointVel = 0;
            errorPos = 0;
            setPointPos = 0;
            tunableNumberInit();
            shuffleboardMagik();
            changeListener();
        } else {
            return;
        }
    }

    public void shuffleboardMagik() {
        if (Constants.tuningMode) {
            SmartDashboard.putNumber("Velocity Error", errorVel);
            SmartDashboard.putNumber("Velocity Set Point", setPointVel);
            SmartDashboard.putNumber("Position Error", errorPos);
            SmartDashboard.putNumber("Position Set Point", setPointPos);
        } else {
            return;
        }
    }

    private void tunableNumberInit() {
        tunableSteerID = new TunableInt("Steer Motor ID", 0);
        tunableDriveID = new TunableInt("Drive Motor ID", 0);
        tunableSteerP = new TunableNumber("Steer Motor P Gain", 0);
        tunableDriveP = new TunableNumber("Drive Motor P Gain", 0);
        tunableSteerI = new TunableNumber("Steer Motor I Gain", 0);
        tunableDriveI = new TunableNumber("Drive Motor I Gain", 0);
        tunableSteerD = new TunableNumber("Steer Motor D Gain", 0);
        tunableDriveD = new TunableNumber("Drive Motor D Gain", 0);
        tunableSteerF = new TunableNumber("Steer Motor F Gain", 0);
        tunableDriveF = new TunableNumber("Drive Motor F Gain", 0);
        tunableHeading = new TunableNumber("Heading set point", 0);
        tunableSpeed = new TunableNumber("Speed set point", 0);
    }

    private void changeListener() {
        tunableSteerID.addChangeListener((val) -> module = new SwerveModuleConfig(driveID, val, 0.0, pSteer,
                iSteer, dSteer, fSteer, pDrive, iDrive, dDrive, fDrive));
        tunableDriveID.addChangeListener((val) -> module = new SwerveModuleConfig(val, steerID, 0.0, pSteer,
                iSteer, dSteer, fSteer, pDrive, iDrive, dDrive, fDrive));
        tunableSteerP.addChangeListener((val) -> module.configSteerPID(val, iSteer, dSteer, fSteer));
        tunableDriveP.addChangeListener((val) -> module.configDrivePID(val, iDrive, dDrive, fDrive));
        tunableSteerI.addChangeListener((val) -> module.configSteerPID(pSteer, val, dSteer, fSteer));
        tunableDriveI.addChangeListener((val) -> module.configDrivePID(pDrive, val, dDrive, fDrive));
        tunableSteerD.addChangeListener((val) -> module.configSteerPID(pSteer, iSteer, val, fSteer));
        tunableDriveD.addChangeListener((val) -> module.configDrivePID(pDrive, iDrive, val, fDrive));
        tunableSteerF.addChangeListener((val) -> module.configSteerPID(pSteer, iSteer, dSteer, val));
        tunableDriveF.addChangeListener((val) -> module.configDrivePID(pDrive, iDrive, dDrive, val));
        tunableHeading.addChangeListener((val) -> module.setSteer(val));
        tunableSpeed.addChangeListener((val) -> module.setDrive(val));
    }

    public void Tune() {
        if (Constants.tuningMode) {
            pSteer = tunableSteerP.get();
            iSteer = tunableSteerI.get();
            dSteer = tunableSteerD.get();
            fSteer = tunableSteerF.get();
            pDrive = tunableDriveP.get();
            iDrive = tunableDriveI.get();
            dDrive = tunableDriveD.get();
            fDrive = tunableDriveF.get();
            driveID = tunableDriveID.get();
            steerID = tunableSteerID.get();
            setPointVel = tunableSpeed.get();
            setPointPos = tunableHeading.get();
            errorVel = setPointVel - module.getDriveVelocity();
            errorPos = setPointPos - module.getAngle();
        }
    }

    public void printFeeds() {
        if (Constants.tuningMode) {
            System.out.println("Steer P Gain" + pSteer);
            System.out.println("Steer I Gain" + iSteer);
            System.out.println("Steer D Gain" + dSteer);
            System.out.println("Steer F Gain" + fSteer);
            System.out.println("Drive P Gain" + pDrive);
            System.out.println("Drive I Gain" + iDrive);
            System.out.println("Drive D Gain" + dDrive);
            System.out.println("Drive F Gain" + fDrive);
        }
    }
}
