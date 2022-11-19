package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;

public class ControllerFactory {

    public XboxController getXboxController(int port) {
        return new XboxController(port);
    }
}
