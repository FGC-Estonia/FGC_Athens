package org.firstinspires.ftc.teamcode;  //place where the code is located

/* Damn,

There is a huge bug in our code:
    ,__                   __
    '~~****Nm_    _mZ*****~~
            _8@mm@K_
           W~@`  '@~W
          ][][    ][][
    gz    'W'W.  ,W`W`    es
  ,Wf    gZ****MA****Ns    VW.
 gA`   ,Wf     ][     VW.   'Ms
Wf    ,@`      ][      '@.    VW
M.    W`  _mm_ ][ _mm_  'W    ,A
'W   ][  i@@@@i][i@@@@i  ][   W`
 !b  @   !@@@@!][!@@@@!   @  d!
  VWmP    ~**~ ][ ~**~    YmWf
    ][         ][         ][
  ,mW[         ][         ]Wm.
 ,A` @  ,gms.  ][  ,gms.  @ 'M.
 W`  Yi W@@@W  ][  W@@@W iP  'W
d!   'W M@@@A  ][  M@@@A W`   !b
@.    !b'V*f`  ][  'V*f`d!    ,@
'Ms    VW.     ][     ,Wf    gA`
  VW.   'Ms.   ][   ,gA`   ,Wf
   'Ms    'V*mmWWmm*f`    gA`
*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mainModules.Alignment;
import org.firstinspires.ftc.teamcode.mainModules.BallPusher;
import org.firstinspires.ftc.teamcode.mainModules.Raising;
import org.firstinspires.ftc.teamcode.mainModules.ImuManager;
import org.firstinspires.ftc.teamcode.mainModules.MoveRobot;
import org.firstinspires.ftc.teamcode.mainModules.Presses;

@TeleOp(name = "Main code Estonia Athens")
// allows to display the code in the driver station, comment out to remove

public class EstoniaAthens extends LinearOpMode { //file name is EstoniaAthens.java    extends the prebuilt LinearOpMode by rev to run
    @Override
    public void runOpMode() {
        boolean protect = true; // activate try/catch to protect the code
        /*
         * map objects
         * objectName = new ClassName()
         * eg:
         * runMotor = new RunMotor();
         *
         * if te external classes require initialisation do it here
         * eg:
         * RunMotor runMotor = new RunMotor(hardwareMap, telemetry);
         */

        ImuManager imuManager = new ImuManager(protect, hardwareMap, telemetry);
        MoveRobot moveRobot = new MoveRobot(protect, hardwareMap, telemetry, false);
        Raising raising = new Raising(protect, hardwareMap, telemetry);
        Alignment alignment = new Alignment(protect, hardwareMap, telemetry, gamepad1, gamepad2);
        BallPusher ballPusher = new BallPusher(protect, hardwareMap, telemetry);

        Presses gamepad1_left_trigger = new Presses(gamepad1);
        Presses gamepad1_right_trigger = new Presses(gamepad1);
        Presses gamepad1_left_bumper = new Presses(gamepad1);
        Presses gamepad1_right_bumper = new Presses(gamepad1);

        Presses.ToggleGroup heightSelectToggleGroup = new Presses.ToggleGroup();
        Presses gamepad2_cross = new Presses(heightSelectToggleGroup, gamepad2);
        Presses gamepad2_triangle = new Presses(heightSelectToggleGroup, gamepad2);
        Presses gamepad2_square = new Presses(heightSelectToggleGroup, gamepad2);
        Presses gamepad2_circle = new Presses(heightSelectToggleGroup, gamepad2);

        Presses.ToggleGroup speedSelectToggle = new Presses.ToggleGroup();
        Presses gamepad1_square = new Presses(speedSelectToggle, gamepad1);
        Presses gamepad1_triangle = new Presses(speedSelectToggle, gamepad1);
        Presses gamepad1_circle = new Presses(speedSelectToggle, gamepad1);
        Presses gamepad1_cross = new Presses(speedSelectToggle, gamepad1);
        gamepad1_triangle.setToggleTrue();//set deafult value

        Presses gamepad2_dpad_left = new Presses(gamepad2);
        Presses gamepad2_dpad_right = new Presses(gamepad2);

        double maxRaisedVelocity;

        telemetry.update();
        waitForStart(); //everything has been initialized, waiting for the start button

        double distance1 = 404.0;
        double distance2 = 1167.0; // 3 calculated distances in mm from sensor to wall to align centre of drivebase to goal
        double distance3 = 1945.0;

        double target = distance1;
        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive()) { // main loop

            // raise
            {
                double raiseManual = gamepad2.right_stick_y;
                boolean goIf = !gamepad2.left_bumper;
                boolean goToBottom = gamepad2_cross.toggle(gamepad2.cross);
                boolean goTo80 = gamepad2_square.toggle(gamepad2.square);
                boolean goTo100 = gamepad2_triangle.toggle(gamepad2.triangle);
                boolean goTo120 = gamepad2_circle.toggle(gamepad2.circle);


                maxRaisedVelocity = raising.raise(
                        raiseManual,
                        goIf,
                        goToBottom,
                        goTo80,
                        goTo100,
                        goTo120
                );
            }


            //gyro reset
            {
                if (gamepad1.right_bumper) {
                    gamepad1.rumble(0.5, 0.5, 1000);
                    imuManager.resetImu();
                }
                //create new imu if old one is fucked
                if (gamepad1_right_trigger.pressed(gamepad1.right_trigger>0.5)){
                    try {
                        imuManager = new ImuManager(protect, hardwareMap, telemetry);
                        gamepad1.rumble(0, 1, 2000);
                    } catch ( Exception e){
                        telemetry.addData("error", e);
                    }
                }
            }

            //move robot
            {
                /*
                // change desired distance
                if (gamepad2.left_bumper) {
                    if (gamepad2.cross) {
                        target = distance1;
                        gamepad1.rumble(0.5, 0.5, 250);
                    } else if (gamepad2.square) {
                        target = distance2;
                        gamepad2.rumble(0.5, 0.5, 250);
                    } else if (gamepad2.triangle) {
                        target = distance3;
                        gamepad2.rumble(0.5, 0.5, 250);
                    }
                }
                */
                //position automatically when pressed
                /*
                double autoCompensation = 0;
                boolean lockToBackWall = false;
*/
                {
                    /*
                    if (gamepad1.right_trigger > 0.5) { // if the right trigger is pressed-auto drive
                        lockToBackWall = true;
                        autoCompensation = alignment.alignTarget(target);
                    }*/

                    double leftRight = gamepad1.left_stick_x;
                    double imuAngle = imuManager.getYawRadians();
                    double frontBack = -gamepad1.left_stick_y;
                    double turn = gamepad1.right_stick_x;
                    boolean fieldCentric = gamepad1_left_trigger.toggle(gamepad1.left_trigger > 0.5);
                    if (gamepad1_left_trigger.pressed(gamepad1.left_trigger > 0.5)) {
                        gamepad1.rumble(1, 1, 250);
                    }
                    telemetry.addData("Field Centric", fieldCentric);
                    boolean turnFieldCentric = gamepad1_left_bumper.toggle(gamepad1.left_bumper);

                    boolean speed1 = gamepad1_cross.toggle(gamepad1.cross);
                    boolean speed2 = gamepad1_square.toggle(gamepad1.square);
                    boolean speed3 = gamepad1_triangle.toggle(gamepad1.triangle);

                    moveRobot.move(
                            imuAngle,
                            frontBack, leftRight, turn,
                            fieldCentric, turnFieldCentric,
                            false, 0,
                            /*lockToBackWall, autoCompensation,*/
                            speed1, speed2, speed3,
                            maxRaisedVelocity
                    );

                    alignment.addToTelemetry();
                }
            }

                // release
                {
                    boolean releaseLeft = gamepad2.left_trigger > 0.5;
                    boolean releaseRight = gamepad2.right_trigger > 0.5;

                    raising.release(
                            releaseLeft,
                            releaseRight
                    );
                }

                // pushing hands
                {
                    boolean leftState = gamepad2_dpad_left.toggle(gamepad2.dpad_left);
                    boolean rightState = gamepad2_dpad_right.toggle(gamepad2.dpad_right);

                    ballPusher.moveHands(
                            leftState,
                            rightState
                    );
                }
                telemetry.update();

        }
    }
}