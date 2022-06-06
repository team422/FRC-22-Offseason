package frc.mechtechsupport.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SwerveModuleMath {
    public static double restrictAngle(double angle) {
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public static final SwerveDriveKinematics swerveMagik = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(Constants.botLengthIn) / 2,
                    -Units.inchesToMeters(Constants.botWidthIn) / 2), //FR
            new Translation2d(Units.inchesToMeters(Constants.botLengthIn) / 2,
                    Units.inchesToMeters(Constants.botWidthIn) / 2), //FL
            new Translation2d(-Units.inchesToMeters(Constants.botLengthIn) / 2,
                    -Units.inchesToMeters(Constants.botWidthIn) / 2), //BR
            new Translation2d(-Units.inchesToMeters(Constants.botLengthIn) / 2,
                    Units.inchesToMeters(Constants.botWidthIn) / 2) //BL
    );

    public static double boundPM180(double angle) {
        while (angle >= 180.0)
            angle -= 360.0;
        while (angle < -180.0)
            angle += 360.0;
        return angle;
    }

    public static double getXHeading(double angle) {
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }

        if (angle >= 0 && angle <= 90) {
            return Math.cos(Math.toRadians(90 - angle));
        } else if (angle >= 270 && angle <= 360) {
            return -Math.cos(Math.toRadians(270 - angle));
        } else if (angle >= 90 && angle <= 270) {
            return Math.cos(-Math.toRadians(angle - 90));
        }

        return 0.0;
    }

    public static double getYHeading(double angle) {
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }

        if (angle >= 0 && angle <= 90) {
            return Math.sin(Math.toRadians(90 - angle));
        } else if (angle >= 270 && angle <= 360) {
            return -Math.sin(Math.toRadians(270 - angle));
        } else if (angle >= 90 && angle <= 270) {
            return Math.sin(-Math.toRadians(angle - 90));
        }
        return 0.0;
    }
}
