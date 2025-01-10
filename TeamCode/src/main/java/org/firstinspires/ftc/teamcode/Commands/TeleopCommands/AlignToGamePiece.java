package org.firstinspires.ftc.teamcode.Commands.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Sensors.ColorDetector;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.opencv.core.Point;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AlignToGamePiece extends CommandBase {

    MecanumDrivetrain mecanumDrivetrain;
    ColorDetector colorDetector;

    Gamepad gamepad;

    public AlignToGamePiece(MecanumDrivetrain mecanumDrivetrain, ColorDetector colorDetector, Gamepad gamepad) {
        this.mecanumDrivetrain = mecanumDrivetrain;
        this.colorDetector = colorDetector;
        this.gamepad = gamepad;

        addRequirements(mecanumDrivetrain);
    }

    @Override
    public void execute() {
        // Define the closest detection
        Point closestDetection = colorDetector.getClosestDetectionPoint();

        // Know if we have a detection and get the closest
        if (closestDetection != null) {
            // align the robot to it
            mecanumDrivetrain.alignToGamePiece(closestDetection);
        }

    }

    @Override
    public boolean isFinished() {
        return !gamepad.right_bumper;
    }

}
