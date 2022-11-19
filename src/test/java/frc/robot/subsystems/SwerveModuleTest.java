package frc.robot.subsystems;

import static org.mockito.Mockito.mock;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModuleTest {

    CANSparkMax driveMotor;
    CANSparkMax turningMotor;
    AnalogEncoder turningCANCoder;
    double turningCANCoderOffsetDegrees;
    RelativeEncoder driveEncoder;
    RelativeEncoder turningEncoder;
    SparkMaxPIDController turningController;
    SparkMaxPIDController driveController;

    SwerveModule swerveModule;

    @Before
    public void setUp() {
        driveMotor = mock(CANSparkMax.class);
        turningMotor = mock(CANSparkMax.class);
        turningCANCoder = mock(AnalogEncoder.class);
        turningCANCoderOffsetDegrees = 0.0;
        driveEncoder = mock(RelativeEncoder.class);
        turningEncoder = mock(RelativeEncoder.class);
        turningController = mock(SparkMaxPIDController.class);
        driveController = mock(SparkMaxPIDController.class);

        Mockito.when(turningCANCoder.get()).thenReturn(0.0);
        Mockito.when(turningEncoder.getPosition()).thenReturn(0.0);
        Mockito.when(driveEncoder.getPosition()).thenReturn(0.0);
        driveEncoder.getPosition();
        Mockito.verify(driveEncoder).getPosition();
        swerveModule = new SwerveModule(driveMotor, turningMotor, turningCANCoder, turningCANCoderOffsetDegrees,
                driveEncoder,
                turningEncoder, turningController, driveController);
    }

    @Test
    public void testSetDesiredState() {
        //Act
        swerveModule.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        //Assert
        Mockito.verify(turningEncoder).getPosition();
        //Mockito.verify(turningController, Mockito.times(1)).setReference(Mockito.any(),
        //       Mockito.any(CANSparkMax.ControlType.class));
    }

    @Test
    public void testGetState() {
        //Act
        swerveModule.getState();
        //Assert
        Mockito.verify(turningEncoder, Mockito.times(1)).getPosition();
        Mockito.verify(driveEncoder, Mockito.times(1)).getPosition();
    }

    @Test
    public void testResetEncoders() {
        //Act
        swerveModule.resetEncoders();
        //Assert
        Mockito.verify(turningEncoder, Mockito.times(1)).setPosition(0.0);
        Mockito.verify(driveEncoder, Mockito.times(1)).setPosition(0.0);
    }

}
