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

        // Arm
        public static String armServo = "armServo";
        public static String wristServo = "wristServo";
        public static String gripperAngleServo = "gripperAngleServo";
        public static String gripperServo = "gripperServo";

        // Slider
        public static String sliderRightMotorId = "sliderRightMotor";
        public static String sliderLeftMotorId = "sliderLeftMotor";

        // Slider angle
        public static String sliderAngleRightMotorId = "sliderAngleRightMotor";
        public static String sliderAngleLeftMotorId = "sliderAngleLeftMotor";

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

    public static final class Arm {
        // Positions
        public static double homePositon = 180.0;
        public static double intakePosition = 40;
        public static double sampleTakePosition = 0.0;
        public static double specimenTakePosition = 180.0;
        public static double specimenScorePosition = 0.0;
        public static double basketScorePosition = 0.0;

    }

    public static final class Wrist {
        // Positions
        public static double homePositon = 180.0;
        public static double intakePosition = 20.0;
        public static double sampleTakePosition = 0.0;
        public static double specimenTakePosition = 0.0;
        public static double specimenScorePosition = 0.0;
        public static double basketScorePosition = 0.0;

    }

    public static final class Gripper {
        // Positions gripper
        public static double openPositon = 180.0;
        public static double closePosition = 0.0;

        // Positions gripper angle
        public static double lateralPosition = 0.0;

    }

    public static final class Slider {
        // Velocity
        public static double maxAngularVelocity = 30;

        // Positions
        public static int homePosition = 0;
        public static int specimenScorePosition = 0;

        public static int basketScorePosition = 0;

        public static double encoderConvertionFactor = 1 / 537.7;
        public static boolean rightEncoderReversed = false;
        public static boolean leftEncoderReversed = false;

        public static double positionTolerance = 0.1;

        public static int upLimit = 0;
        public static int downLimit = 0;
    }

    public static final class SliderAngle {
        // Velocity
        public static double maxAngularVelocity = 30;
        public static double positionTolerance = 0.1;
        public static double upLimit = 89.0;
        public static double downLimit = 1.0;

        // Encoder
        public static double ticksPerRev = 1.0 / 8192.0;


        // Positions
        public static double homePosition = 180.0;
        public static double intakePosition = 1.0;

        public static double specimenScorePosition = 0;

        public static double basketScorePosition = 0;
    }


}

