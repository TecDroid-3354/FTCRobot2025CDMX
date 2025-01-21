package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants.FConstants;
import org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Auto", group = "Examples")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Leave pices points
    private final Pose startPose = new Pose(0.0, -60.0, Math.toRadians(90.0));
    private final Pose score1 = new Pose(-0.0, -37.0, Math.toRadians(90));
    private final Pose cruve1 = new Pose(32, -40, Math.toRadians(90));
    private final Pose goToPiece1 = new Pose(37, -8, Math.toRadians(90));
    private final Pose curve2 = new Pose(43, -18, Math.toRadians(90));
    private final Pose leavePiece1 = new Pose(43, -42, Math.toRadians(90));
    private final Pose goToPiece2 = new Pose(45, -8, Math.toRadians(90));
    private final Pose goRight = new Pose(48, -8, Math.toRadians(90));

    private final Pose leavePiece2 = new Pose(49, -42, Math.toRadians(90));

    // Pick smaple 1
    private final Pose alignToSample = new Pose(45, -39, Math.toRadians(270));



    private Path scorePreload;
    private PathChain leavePiecePC;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(score1)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1.getHeading());

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        leavePiecePC = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1), new Point(cruve1)))
                .setLinearHeadingInterpolation(score1.getHeading(), cruve1.getHeading())
                .addPath(new BezierCurve(new Point(cruve1), new Point(goToPiece1)))
                .setLinearHeadingInterpolation(cruve1.getHeading(), goToPiece1.getHeading())
                .addPath(new BezierCurve(new Point(goToPiece1), new Point(curve2)))
                .setLinearHeadingInterpolation(goToPiece1.getHeading(), curve2.getHeading())
                .addPath(new BezierLine(new Point(curve2), new Point(leavePiece1)))
                .setLinearHeadingInterpolation(curve2.getHeading(), leavePiece1.getHeading())
                .addPath(new BezierLine(new Point(leavePiece1), new Point(goToPiece2)))
                .setLinearHeadingInterpolation(leavePiece1.getHeading(), goToPiece2.getHeading())
                .addPath(new BezierLine(new Point(goToPiece2), new Point(goRight)))
                .setLinearHeadingInterpolation(goToPiece2.getHeading(), goRight.getHeading())
                .addPath(new BezierCurve(new Point(goRight), new Point(leavePiece2)))
                .setLinearHeadingInterpolation(goRight.getHeading(), leavePiece2.getHeading())
                .addPath(new BezierCurve(new Point(leavePiece2), new Point(alignToSample)))
                .setLinearHeadingInterpolation(leavePiece2.getHeading(), alignToSample.getHeading())
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(leavePiecePC,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }


}

