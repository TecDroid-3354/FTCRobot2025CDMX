package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Commands.TeleopCommands.AlignToGamePiece;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.JoystickCMD;
import org.firstinspires.ftc.teamcode.Sensors.ColorDetector;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.vision.opencv.ColorRange;

@TeleOp(name = "CMD", group = "Op Mode")
public class CMDOpMode extends CommandOpMode {
    MecanumDrivetrain mecanumDrivetrain;
    ColorDetector colorDetector;
    IMU imu;
    SparkFunOTOS otos;
    GamepadEx controller;

    @Override
    public void initialize() {
        imu = hardwareMap.get(IMU.class, Ids.imuId);
        //otos = hardwareMap.get(SparkFunOTOS.class, "OTOS");

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, telemetry);
        mecanumDrivetrain.setDefaultCommand(new JoystickCMD(
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.right_stick_x,
                () -> gamepad1.right_stick_button,
                this::getGyroYaw,
                mecanumDrivetrain
        ));

        colorDetector = new ColorDetector(hardwareMap, telemetry, ColorRange.BLUE);

        // Gamepad buttons
        controller = new GamepadEx(gamepad1);

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

        // align to a game pice
        new GamepadButton(controller, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new AlignToGamePiece(mecanumDrivetrain, colorDetector, gamepad1));
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
            if (colorDetector.getClosestDetectionPoint() != null){
                telemetry.addData("ClosestDetection x:", colorDetector.getClosestDetectionPoint().x);
                telemetry.addData("ClosestDetection y:", colorDetector.getClosestDetectionPoint().y);
            }
            colorDetector.colorDetectionsData();
            telemetry.update();

            run();
        }
        reset();
    }
}
