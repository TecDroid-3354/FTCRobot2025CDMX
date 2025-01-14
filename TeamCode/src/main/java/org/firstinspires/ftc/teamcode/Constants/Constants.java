package org.firstinspires.ftc.teamcode.Constants;

import org.opencv.core.Mat;

public class Constants {
    public static final class Velocities {
        // Meccanum drive
        public static final double driveMaxVelocity = 0.5;
        public static final double angularMaxVelocity = 0.5;

    }

    public static final class Ids {
        // Drivetrain
        public static String frontLeftId = "frontLeft";
        public static String frontRightId = "frontRight";
        public static String backLeftId = "backLeft";
        public static String backRightId = "backRight";

        // Imu
        public static String imuId = "imu";

        // OTOS
        public static String otosId = "OTOS";

        // Webcam
        public static String webcamName = "webcam0";

    }

    public static final class MecanumConstants {
        public static double ticksPerRevolution = 425;
        public static double kGearRatio = 15.0;
        public static double wheelRadius = 0.075;
        public static double kTicks2Rot = 1 / ticksPerRevolution;
        public static double kMps2Radps = 1 / wheelRadius;
        public static double kTicksps2mps = (2 * Math.PI * wheelRadius) / ticksPerRevolution;

        public static double maxMetersPerSecondDrive = 2.0;
        public static double maxMetersPerSecondRotation = 2.0;

        // PIDF coeficients

        public static double kFrontLeftF = 18.5;
        public static double kFrontRightF = 18.5;
        public static double kBackLeftF = 17.0;
        public static double kBackRightF = 17.0;

    }
}

