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
import org.firstinspires.ftc.teamcode.Commands.TeleopCommands.TakeSampleCommand;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.JoystickCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;

@TeleOp(name = "CMD", group = "Op Mode")
public class CMDOpMode extends CommandOpMode {
    MecanumDrivetrain mecanumDrivetrain;

    boolean hasScoreAPiece = false;
    IMU imu;
    SparkFunOTOS otos;
    GamepadEx controller;

    SliderAngle sliderAngle;
    Slider slider;
    Gripper gripper;
    GripperAngle gripperAngle;
    Arm arm;
    Wrist wrist;
    JoystickCMD joystickCMD;
    TakeSampleCommand takeSampleCommand;

    // toggle variable
    boolean btnPressed = false;

    @Override
    public void initialize() {
        imu = hardwareMap.get(IMU.class, Ids.imuId);
        imu.resetYaw();

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.resetTracking();

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, telemetry);
        joystickCMD = new JoystickCMD(
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.right_stick_x,
                () -> gamepad1.right_stick_button,
                this::getGyroYaw,
                mecanumDrivetrain
        );

        mecanumDrivetrain.setDefaultCommand(joystickCMD);

        sliderAngle = new SliderAngle(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);

        gripper = new Gripper(hardwareMap, telemetry);
        gripperAngle = new GripperAngle(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);

        takeSampleCommand = new TakeSampleCommand(arm, slider, sliderAngle, gripper, gripperAngle, wrist);

        // Gamepad buttons
        controller = new GamepadEx(gamepad1);

        configureButtonBindings();
    }
    public void configureButtonBindings() {
        // reset IMU
        new GamepadButton(controller, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> {
                    otos.resetTracking();
                    imu.resetYaw();
                }));

        // Open gripper
        new GamepadButton(controller, GamepadKeys.Button.B)
                .whenPressed(gripper::open)
                .whenReleased(gripper::close);

        // Change gripper orientation
        new GamepadButton(controller, GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(gripperAngle::goToLateralPosition);

        new GamepadButton(controller, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(gripperAngle::goToVerticalPosition);

        // Take sample

        new GamepadButton(controller, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(takeSampleCommand);

    }

    public double getGyroYaw() {
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

            // Quesadilla position
            if (controller.getButton(GamepadKeys.Button.A) && slider.getLeftEncoderPosition() < 3.0) {
                arm.goToPosition(Constants.Arm.homePositon);
                wrist.goToPosition(Constants.Wrist.homePositon);
                sliderAngle.goToQuesadillaPosition();
                slider.goToHomePosition();
                gripperAngle.goToIntakePosition();
                gripper.close();

                // change velocity
                joystickCMD.setVelocityFactor(1.0);
            }
            // Intake secuence
            if (controller.getButton(GamepadKeys.Button.LEFT_BUMPER)){
                arm.goToPosition(Constants.Arm.intakePosition);
                wrist.goToPosition(Constants.Wrist.intakePosition);
                sliderAngle.goToIntakePosition();
                slider.goToIntakePosition();
                gripper.open();

                // slow down the velocity
                joystickCMD.setVelocityFactor(0.7);
            }

            // go to home position
            if (controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0){
                if (!sliderAngle.isAtPosition(Constants.SliderAngle.homePosition)) {
                    sliderAngle.goToHomePosition();
                } else {
                    slider.goToHomePosition();
                    gripperAngle.goToIntakePosition();
                    gripper.close();
                }

                hasScoreAPiece = false;
            }

            if (toggleBasketState(controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0)) {
                basketScore();
            }

            telemetry.update();

            run();
        }
        reset();
    }

        public void basketScore() {
            if (!hasScoreAPiece) {
                if (!slider.isAtPosition(Constants.Slider.basketScorePosition)) {
                    arm.goToPosition(Constants.Arm.basketScorePosition);
                    wrist.goToPosition(Constants.Wrist.basketScorePosition);

                    if (sliderAngle.getAbsoluteEncoderPosition() > 85) {
                        slider.goToBasketPosition();
                    } else {
                        sliderAngle.goToHomePosition();
                    }
                } else {
                    sliderAngle.goToBasketPosition();
                    if (sliderAngle.isAtPosition(Constants.SliderAngle.basketScorePosition)) {
                        hasScoreAPiece = true;
                    }

                }
            } else {
                if (!sliderAngle.isAtPosition(Constants.SliderAngle.homePosition)) {
                    sliderAngle.goToHomePosition();
                } else {
                    slider.goToHomePosition();

                    if (slider.isAtPosition(Constants.Slider.homePosition)) {
                        hasScoreAPiece = false;
                    }
                }
            }

    }

    public boolean toggleBasketState(boolean btn) {
        if (btn && !btnPressed){
            btnPressed = true;
            return true;
        }

        if (!btn) {
            btnPressed = false;
        }

        return false;
    }
}
