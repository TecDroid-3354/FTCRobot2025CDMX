package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.TeleopCommands.AlignToGamePiece;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.JoystickCMD;
import org.firstinspires.ftc.teamcode.Sensors.ColorDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

@TeleOp(name = "CMD", group = "Op Mode")
public class CMDOpMode extends CommandOpMode {
    MecanumDrivetrain mecanumDrivetrain;
    ColorDetector colorDetector;
    IMU imu;
    SparkFunOTOS otos;
    GamepadEx controller;

    SliderAngle sliderAngle;
    Slider slider;
    Gripper gripper;
    GripperAngle gripperAngle;
    Arm arm;
    Wrist wrist;

    @Override
    public void initialize() {
        imu = hardwareMap.get(IMU.class, Ids.imuId);

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.resetTracking();

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, telemetry);
        mecanumDrivetrain.setDefaultCommand(new JoystickCMD(
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.right_stick_x,
                () -> gamepad1.right_stick_button,
                this::getGyroYaw,
                mecanumDrivetrain
        ));

        sliderAngle = new SliderAngle(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);

        gripper = new Gripper(hardwareMap, telemetry);
        gripperAngle = new GripperAngle(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);

        colorDetector = new ColorDetector(hardwareMap, telemetry, ColorRange.BLUE);

        // Gamepad buttons
        controller = new GamepadEx(gamepad1);

        // initial positions
        //wrist.goToPosition(Constants.Wrist.intakePosition);

        configureButtonBindings();
    }
    public void configureButtonBindings() {
        // reset IMU
        new GamepadButton(controller, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> {
                    imu.resetYaw();
                }));

        new GamepadButton(controller, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {
                    imu.resetYaw();
                }));
    }

    public double getGyroYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Yaw", getGyroYaw());
            /*colorDetector.colorDetectionsData();
            if (colorDetector.getClosestDetectionPoint() != null){
                telemetry.addData("ClosestDetection x:", colorDetector.getClosestDetectionPoint().x);
                telemetry.addData("ClosestDetection y:", colorDetector.getClosestDetectionPoint().y);

                ColorBlobLocatorProcessor.Blob closestDetection = colorDetector.getClosestDetection();
                telemetry.addData("Orientation", colorDetector.getOrientationOfPiece(closestDetection.getBoxFit().angle, closestDetection.getAspectRatio()));
            }*/
            //sliderAngle.encoderData();
            //slider.motorsData();
            //gripper.servoInfo();
            //gripperAngle.servoInfo();
            arm.servoInfo();
            wrist.servoInfo();

            if (controller.getButton(GamepadKeys.Button.LEFT_BUMPER)){
                arm.goToPosition(Constants.Arm.intakePosition);
                wrist.goToPosition(Constants.Wrist.intakePosition);
                sliderAngle.goToIntakePosition();
                gripper.open();
            }if (controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0){
                sliderAngle.goToIntakePosition();
                arm.goToPosition(Constants.Arm.sampleTakePosition);
                wrist.goToPosition(Constants.Wrist.intakePosition);
                gripper.close();
                try {Thread.sleep(500);} catch (InterruptedException e) {}
                arm.goToPosition(Constants.Arm.intakePosition);

            } if (controller.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                arm.goToPosition(Constants.Arm.homePositon);
                wrist.goToPosition(Constants.Wrist.homePositon);
                sliderAngle.goToHomePosition();
                gripper.close();
            }

            gripper.openClose(controller.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON));

            telemetry.update();

            run();
        }
        reset();
    }
}
