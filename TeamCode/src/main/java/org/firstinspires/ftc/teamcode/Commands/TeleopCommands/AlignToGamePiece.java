package org.firstinspires.ftc.teamcode.Commands.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Sensors.ColorDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.opencv.core.Point;

public class AlignToGamePiece extends CommandBase {

    MecanumDrivetrain mecanumDrivetrain;
    ColorDetector colorDetector;
    private boolean isAligned = false;

    GamepadEx gamepad;
    Slider slider;
    Gripper gripper;
    GripperAngle gripperAngle;
    Arm arm;
    Wrist wrist;

    public AlignToGamePiece(MecanumDrivetrain mecanumDrivetrain, ColorDetector colorDetector, GamepadEx gamepad,
                            Arm arm,
                            Slider slider,
                            Gripper gripper,
                            GripperAngle gripperAngle,
                            Wrist wrist) {
        this.mecanumDrivetrain = mecanumDrivetrain;
        this.colorDetector = colorDetector;
        this.gamepad = gamepad;

        this.arm = arm;
        this.slider = slider;
        this.gripper = gripper;
        this.gripperAngle = gripperAngle;
        this.wrist = wrist;

        addRequirements(mecanumDrivetrain);
    }

    @Override
    public void execute() {
        // Define the closest detection
        Point closestDetection = colorDetector.getClosestDetectionPoint();

        // Know if we have a detection and get the closest
        if (closestDetection != null) {
            // align the robot to it
            try {
                if (mecanumDrivetrain.alignToGamePiece(closestDetection)){
                    arm.goToPosition(Constants.Arm.sampleTakePosition);
                    wrist.goToPosition(Constants.Wrist.intakePosition);
                    slider.goToIntakePosition();
                    gripperAngle.goToLateralPosition();
                    gripper.close();
                    try {Thread.sleep(500);} catch (InterruptedException e) {}
                    arm.goToPosition(Constants.Arm.intakePosition);
                }
            } catch (RuntimeException e) {}
        }

    }

    @Override
    public boolean isFinished() {
        return !gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER);
    }

}
