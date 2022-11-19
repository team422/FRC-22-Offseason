package frc.robot.commands;

import org.junit.Before;

import frc.robot.utils.MotorFactory;

public class DriveOneModuleTest {
    MockMotorFactory motorFactory;

    private class MockMotorFactory extends MotorFactory {
        public MockMotorFactory() {
        }

    }

    @Before
    public void setUp() {
        motorFactory = new MockMotorFactory();
    }

}
