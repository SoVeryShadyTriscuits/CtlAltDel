package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive", group="Test")
// @Disabled
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;
public class Drive extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDefinition robot = new HardwareDefinition();

    @Override
    public void runOpMode() throws InterruptedException {
        double move = 0;
        double side;
        double max;
        double rotate;
        double MaxSpeed = 1;
        boolean fPrevState = false;
        boolean fCurrState = false;
        boolean liftUp = false;
        boolean liftDown = false;
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean armDown = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Drive code launch succesful" +
                "; also you spelled successful wrong");
        telemetry.update();

            robot.dropLiftLeft.setPosition(0);
            robot.dropLiftRight.setPosition(1);





        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            if(gamepad1.left_trigger > 0){
                MaxSpeed = 1;
            }
            if(gamepad1.right_trigger > 0){
                MaxSpeed = 0.2;
            }
            move = -gamepad1.left_stick_y;
            side = -gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;
            // Normalize the values so none exceed +/- 1.0
            max = Math.max(Math.abs(move), Math.abs(side));
            max = Math.max(Math.abs(max), Math.abs(rotate));
            if (max > MaxSpeed) {
                move *= MaxSpeed;
                side *= MaxSpeed;
                rotate *= MaxSpeed;
            }
            telemetry.addLine();
            telemetry.addData("Left Y", gamepad1.left_stick_y);
            telemetry.addData("Left X", gamepad1.left_stick_x);
            telemetry.addData("Right X", gamepad1.left_stick_x);
            telemetry.addData("move", move);
            telemetry.addData("side", side);
            telemetry.addData("rotate", rotate);
            telemetry.update();


            robot.leftFrontWheel.setPower(move + side + rotate);
            robot.leftBackWheel.setPower(move - side + rotate);
            robot.rightFrontWheel.setPower(move - side - rotate);
            robot.rightBackWheel.setPower(move + side - rotate);


            if (gamepad1.y == true) {
                robot.dropLiftLeft.setPosition(1);
                robot.dropLiftRight.setPosition(0);
                armDown = true;
            }

            fCurrState = gamepad1.right_bumper;
            if ((fCurrState == true) && (fCurrState != fPrevState)) { //

                liftUp = !liftUp;
            }

            fPrevState = fCurrState;

            bCurrState = gamepad1.left_bumper;
            if ((bCurrState == true) && (bCurrState != bPrevState)) {

                liftDown = !liftDown;
            }

            bPrevState = bCurrState;
            if ((liftUp == true) && (armDown == true)) {
                robot.liftLeft.setPower(0.5);
                robot.liftRight.setPower(-0.5);
            } else {
                if ((liftDown == true) && (armDown == true)) {
                    robot.liftLeft.setPower(-0.5);
                    robot.liftRight.setPower(0.5);
                } else {
                    robot.liftLeft.setPower(0);
                    robot.liftRight.setPower(0);
                }
            }


            if (gamepad1.x == true) {
                robot.beacon.setPosition(0.3);
            } else {
                if (gamepad1.b == true) {
                    robot.beacon.setPosition(0.7);
                } else {
                    robot.beacon.setPosition(0.5);
                }
            }
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}






