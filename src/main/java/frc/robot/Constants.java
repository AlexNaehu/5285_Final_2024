package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.14;
        public static final double kTurningMotorGearRatio = 14 / 50;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5; //guess 0.5
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22.75);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),//fl
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),//fr
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),//bl
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));//br

        public static final int kFrontLeftDriveMotorPort = 23;
        public static final int kBackLeftDriveMotorPort = 25;
        public static final int kFrontRightDriveMotorPort = 15;
        public static final int kBackRightDriveMotorPort = 28;

        public static final int kFrontLeftTurningMotorPort = 22;
        public static final int kBackLeftTurningMotorPort = 26;
        public static final int kFrontRightTurningMotorPort = 14;
        public static final int kBackRightTurningMotorPort = 27;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 0;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -152.278057*Math.PI/180;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (17.822375)*Math.PI/180;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (-42.753406)*Math.PI/180;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -46.417145*Math.PI/180;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 7;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 4 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 3;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;//was 3
    }

    public static final class ArmConstants {
        public static final double INVALID_ANGLE = 245.0; //TODO: Check invalid angle
        public static final double CONTROLLER_INPUT_WAIT_TIME = 0.005;
        public static final int NAVX_RESET_WAIT_TIME = 1;

        public static final double PICK_UP_ANGLE = 0;
        public static final double LOW_SCORE_ANGLE = 100;
        public static final double LOAD_SHOOTER_ANGLE = 180;//TODO: check
        //public static final double CLIMB_ANGLE = 200; //TODO: Need a climbing angle (could just be low score angle)

    }

    public static final class WristConstants {
        public static final double INVALID_ANGLE = 245.0; //TODO: Check invalid angle
        public static final double CONTROLLER_INPUT_WAIT_TIME = 0.005;

        public static final double PICK_UP_ANGLE = 0;
        public static final double LOW_SCORE_ANGLE = 180;
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 1.3;
        public static final double kPXController = 1.5;//1.5 //TODO: May need to tune
        public static final double kPYController = 1.5;//1.5 //TODO: May need to tune
        public static final double kPThetaController = 7;//3 

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }
}
