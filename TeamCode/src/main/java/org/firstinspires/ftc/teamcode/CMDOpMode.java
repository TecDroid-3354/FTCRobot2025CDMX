package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.TeleopCommands.AlignToGamePiece;
import org.firstinspires.ftc.teamcode.Commands.TeleopCommands.BasketScoreSecuence;
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
import org.firstinspires.ftc.vision.opencv.ColorRange;

@TeleOp(name = "CMD", group = "Op Mode")
public class CMDOpMode extends CommandOpMode {
    MecanumDrivetrain mecanumDrivetrain;
    ColorDetector colorDetector;
    IMU imu;
    SparkFunOTOS otos;
    GamepadEx controller;

    GamepadEx controller2;

    SliderAngle sliderAngle;
    Slider slider;
    Gripper gripper;
    GripperAngle gripperAngle;
    Arm arm;
    Wrist wrist;
    BasketScoreSecuence basketScoreSecuence;

    // Commands
    private AlignToGamePiece alignToGamePiece;

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
        basketScoreSecuence = new BasketScoreSecuence(arm, slider, sliderAngle, gripper, gripperAngle, wrist);

        // Gamepad buttons
        controller = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        imu.resetYaw();

        configureButtonBindings();
    }
    public void configureButtonBindings() {
        // reset IMU
        new GamepadButton(controller, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> {
                    imu.resetYaw();
                    otos.resetTracking();
                }));

        new GamepadButton(controller, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {
                    imu.resetYaw();
                    otos.resetTracking();
                }));

        // Open gripper
        new GamepadButton(controller2, GamepadKeys.Button.B)
                .whenPressed(gripper::open)
                .whenReleased(gripper::close);

        // Change gripper orientation
        new GamepadButton(controller2, GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(gripperAngle::changeOrientation);

        // Score specimen orientation
        /*new GamepadButton(controller, GamepadKeys.Button.A)
                .whenPressed(
                        new Runnable() {
                            @Override
                            public void run() {
                                wrist.goToPosition(Constants.Wrist.intakePosition);
                                try {Thread.sleep(500);} catch (InterruptedException e) {}
                                gripper.open();
                                wrist.goToPosition(Constants.Wrist.specimenScorePosition);
                            }
                        }

                );*/
    }

    public double getGyroYaw() {
        //return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return otos.getPosition().h;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            sliderAngle.encoderData();
            slider.motorsData();

            // Intake secuence
            if (controller.getButton(GamepadKeys.Button.LEFT_BUMPER)){
                arm.goToPosition(Constants.Arm.intakePosition);
                wrist.goToPosition(Constants.Wrist.intakePosition);
                sliderAngle.goToIntakePosition();
                slider.goToIntakePosition();
                gripper.open();
            }if (controller.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                arm.goToPosition(Constants.Arm.sampleTakePosition);
                wrist.goToPosition(Constants.Wrist.intakePosition);
                slider.goToIntakePosition();
                gripper.close();
                try {Thread.sleep(500);} catch (InterruptedException e) {}
                arm.goToPosition(Constants.Arm.intakePosition);
                sliderAngle.goToPosIntakePosition();
            }

            // go to home position
            if (controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0){
                arm.goToPosition(Constants.Arm.homePositon);
                wrist.goToPosition(Constants.Wrist.homePositon);
                sliderAngle.goToHomePosition();
                slider.goToHomePosition();
                gripperAngle.goToIntakePosition();
                gripper.close();
            }


            /*else if (controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0) {
                arm.goToPosition(Constants.Arm.specimenScorePosition);
                wrist.goToPosition(Constants.Wrist.specimenScorePosition);
                sliderAngle.goToSpecimenPosition();
                slider.goToSpecimenPosition();
                gripperAngle.goToSpecimenScorePosition();
                gripper.close();
            }*/

            else if (controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0) {
                basketScoreSecuence.schedule();
            }


            telemetry.update();

            run();
        }
        reset();
    }
}
