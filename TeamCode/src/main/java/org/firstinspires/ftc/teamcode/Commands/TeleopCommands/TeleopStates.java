package org.firstinspires.ftc.teamcode.Commands.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Sensors.ColorDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.vision.opencv.ColorRange;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopStates extends CommandBase {

    private final GamepadEx gamepadEx;

    // Subsystems
    private final Arm arm;
    private final Wrist wrist;
    private final Gripper gripper;
    private final GripperAngle gripperAngle;
    private final Slider slider;
    // Sensors
    private final ColorDetector colorDetector;

    // Commands
    private final AlignToGamePiece alignToGamePiece;


    // States
    private boolean basketState = false;
    private boolean specimenState = true;

    public TeleopStates(Gamepad gamepad, HardwareMap hardwareMap, Telemetry telemetry, MecanumDrivetrain mecanumDrivetrain) {
        gamepadEx = new GamepadEx(gamepad);

        arm = new Arm(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);
        gripper = new Gripper(hardwareMap, telemetry);
        gripperAngle = new GripperAngle(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);

        colorDetector = new ColorDetector(hardwareMap, telemetry, ColorRange.BLUE);
        alignToGamePiece = new AlignToGamePiece(mecanumDrivetrain, colorDetector, gamepadEx);

        configureButtonBindings();
    }

    public void configureButtonBindings() {
        // open gripper
        new GamepadButton(gamepadEx, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(gripper::open));

        // home position
        new GamepadButton(gamepadEx, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    arm.goToPosition(Constants.Arm.homePositon);
                    wrist.goToPosition(Constants.Wrist.homePositon);
                    gripperAngle.goToLateralPosition();
                }));
    }

    public void toggleState() {
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)){
            basketState = true;
            specimenState = false;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            basketState = false;
            specimenState = true;
        }
    }


    @Override
    public void execute() {
        toggleState();

        // !!!! Sample Intake !!!!
        if (gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            arm.goToPosition(Constants.Arm.intakePosition);
            wrist.goToPosition(Constants.Wrist.intakePosition);
            gripper.open();
            gripperAngle.goToLateralPosition();
            slider.goToHomePosition();

            // Align to game piece
            //alignToGamePiece.execute();

        } else if (gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0) {
            arm.goToPosition(Constants.Arm.sampleTakePosition);

            // Know if the arm is at the desired position
            if (arm.isAtPosition(Constants.Arm.sampleTakePosition)) {
                gripper.close();
            }

            // Know if the arm and the gripper are at the desired position
            if (arm.isAtPosition(Constants.Arm.sampleTakePosition) && gripper.isClosed()) {
                arm.goToPosition(Constants.Arm.homePositon);
                wrist.goToPosition(Constants.Wrist.homePositon);
            }
        }

        // !!!! Specimen State !!!!
        if (specimenState) {
            // !!!! Specimen Intake !!!!
            if (gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                arm.goToPosition(Constants.Arm.homePositon);
                wrist.goToPosition(Constants.Wrist.homePositon);
                gripper.open();
                gripperAngle.goToLateralPosition();
                slider.goToHomePosition();

                // Align to game piece
                //alignToGamePiece.execute();

            } else if (gamepadEx.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
                gripper.close();
            }

            // !!!! Specimen Score !!!!
            if (gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0){
                arm.goToPosition(Constants.Arm.specimenScorePosition);
                wrist.goToPosition(Constants.Wrist.specimenScorePosition);
                gripper.close();
                gripperAngle.goToLateralPosition();
                slider.goToSpecimenPosition();
            }
        }
        // !!!! Basket State !!!!
        if (basketState) {
            // !!!! Yellow Sample Score !!!!
            if (gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0){
                arm.goToPosition(Constants.Arm.basketScorePosition);
                wrist.goToPosition(Constants.Wrist.basketScorePosition);
                gripper.close();
                gripperAngle.goToLateralPosition();
                slider.goToBasketPosition();
            }
        }
    }
}