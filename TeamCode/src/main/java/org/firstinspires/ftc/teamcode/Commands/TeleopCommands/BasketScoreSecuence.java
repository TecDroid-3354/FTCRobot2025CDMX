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

public class basketPositionSecuence extends CommandBase {
    private boolean isAtPosition = false;
    Slider slider;
    SliderAngle sliderAngle;
    Gripper gripper;
    GripperAngle gripperAngle;
    Arm arm;
    Wrist wrist;

    public basketPositionSecuence(MecanumDrivetrain mecanumDrivetrain, ColorDetector colorDetector,
                            Arm arm,
                            Slider slider,
                            SliderAngle sliderAngle,
                            Gripper gripper,
                            GripperAngle gripperAngle,
                            Wrist wrist) {

        this.arm = arm;
        this.slider = slider;
        this.gripper = gripper;
        this.gripperAngle = gripperAngle;
        this.sliderAngle = sliderAngle;
        this.wrist = wrist;

    }

    @Override
    public void execute() {
        arm.goToPosition(Constants.Arm.homePositon);
        wrist.goToPosition(Constants.Wrist.homePositon);
        sliderAngle.goToHomePosition();
        if (sliderAngle.isAtPosition(Constants.SliderAngle.homePosition)) {
            slider.goToBasketPosition();
            arm.goToPosition(Constants.Arm.basketScorePosition);
            wrist.goToPosition(Constants.Wrist.basketScorePosition);
            isAtPosition = true;
        }

    }

    @Override
    public boolean isFinished() {
        return isAtPosition;
    }

}
