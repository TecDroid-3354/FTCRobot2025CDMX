package org.firstinspires.ftc.teamcode.Constants;

public class Constants {
    public final class Velocities {
        // Meccanum drive
        public final double driveMaxVelocity = 2.0;
        public final double angularMaxVelocity = 2.0;

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
        public static String rightArmServo = "rightArmServo";
        public static String leftArmServo = "leftArmServo";
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

        public static double positionTolerance = 1.0;

    }

    public static final class Arm {
        // Positions
        public static double homePositon = 170.0;
        public static double intakePosition = 30;
        public static double sampleTakePosition = 0.0;
        public static double specimenTakePosition = 180.0;
        public static double specimenScorePosition = 20.0; // 10
        public static double basketScorePosition = 0.0;

    }

    public static final class Wrist {
        // Positions
        public static double homePositon = 180.0;
        public static double intakePosition = 0.0;
        public static double sampleTakePosition = 0.0;
        public static double specimenTakePosition = 0.0;
        public static double specimenScorePosition = 110.0;
        public static double basketScorePosition = 0.0;

    }

    public static final class Gripper {
        // Positions gripper
        public static double openPositon = 160;
        public static double closePosition = 112;

        // Positions gripper angle
        public static double lateralPosition = 15.0;
        public static double verticalPosition = 55; // 75

        public static double intakePosition = 0.0;
        public static double specimenScorePositon = 130.0;

    }

    public static final class Slider {
        // Velocity
        public static double maxAngularVelocity = 30;

        // Positions
        public static double homePosition = 0.1;
        public static double intakePosition = 2.0;

        public static double specimenScorePosition = 1.2; // 2.0

        public static double basketScorePosition = 7.2;

        public static double encoderConversionFactor = 1 / 537.7;
        public static boolean rightEncoderReversed = true;
        public static boolean leftEncoderReversed = true;

        public static double positionTolerance = 0.05;

        public static int upLimit = 0;
        public static int downLimit = 0;
    }

    public static final class SliderAngle {
        // Velocity
        public static double maxAngularVelocity = 30;
        public static double positionTolerance = 0.0;
        public static double upLimit = 90;
        public static double downLimit = 1.0;

        // Encoder
        public static double ticksPerRev = 1.0 / 8192.0;


        // Positions
        public static double homePosition = 89; // 550.0
        public static double intakePosition = 3.0; // 55.0
        public static double quesadillaPosition = 6.0;
        public static double posIntakePosition = 0.0; // 55.0

        public static double specimenScorePosition = 35;

        public static double basketScorePosition = 77;
    }


}

