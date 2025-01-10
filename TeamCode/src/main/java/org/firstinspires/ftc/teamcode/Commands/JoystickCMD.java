package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.opencv.core.Point;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class JoystickCMD extends CommandBase {

    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier rx;
    private DoubleSupplier robotHeading;

    // field oriented toggle
    private BooleanSupplier fieldOrientedBtn;
    private boolean btnPressed = false;
    private boolean fieldOrientedActived = true;

    private MecanumDrivetrain mecanumDrivetrain;
    private IMU imu;

    public JoystickCMD(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rx, BooleanSupplier fieldOrientedBtn,
                       DoubleSupplier robotHeading, MecanumDrivetrain mecanumDrivetrain) {
        this.x = x;
        this.y = y;
        this.rx = rx;
        this.robotHeading = robotHeading;
        this.fieldOrientedBtn = fieldOrientedBtn;

        this.mecanumDrivetrain = mecanumDrivetrain;


        addRequirements(mecanumDrivetrain);
    }

    public void toggleFieldOriented() {
        if (fieldOrientedBtn.getAsBoolean() && !btnPressed){
            btnPressed = true;
            fieldOrientedActived = !fieldOrientedActived;
        }

        if (!fieldOrientedBtn.getAsBoolean()) {
            btnPressed = false;
        }
    }


    @Override
    public void execute() {
        toggleFieldOriented();

        double xVelocity = x.getAsDouble();
        double yVelocity = y.getAsDouble();
        double rxVelocity = rx.getAsDouble();
        double robotYaw = robotHeading.getAsDouble();

        if (fieldOrientedActived) {
            mecanumDrivetrain.driveFieldOriented(xVelocity, yVelocity, rxVelocity, robotYaw);
        }else {
            mecanumDrivetrain.basicDrive(xVelocity, yVelocity, rxVelocity);
        }

    }
}