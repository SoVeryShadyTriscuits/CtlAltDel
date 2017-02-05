package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
/*
  This is NOT an opmode.

  This class can be used to define all the specific hardware for a single robot.
  In this case that robot is a K9 robot.

  This hardware class assumes the following device names have been configured on the robot:
  Note:  All names are lower case and some have single spaces between words.

  Motor channel:  Left  drive motor:        "left motor"
  Motor channel:  Right drive motor:        "right motor"
  Servo channel:  Servo to raise/lower arm: "arm"
  Servo channel:  Servo to open/close claw: "claw"

  Note: the configuration of the servos is such that:
    As the arm servo approaches 0, the arm position moves up (away from the floor).
    As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareDefinition
{
    /* Public OpMode members. */
    public DcMotor  leftFrontWheel    = null;
    public DcMotor  leftBackWheel     = null;
    public DcMotor  rightFrontWheel   = null;
    public DcMotor  rightBackWheel    = null;
    public DcMotor  sweeper           = null;
    public Servo    beacon            = null;
    public DcMotor  liftRight         = null;
    public DcMotor  liftLeft          = null;
    public Servo    dropLiftRight     = null;
    public Servo    dropLiftLeft      = null;
//    public Servo    servo1         = null;
  //  public Servo    servo2         = null;
    //public Servo    servo3         = null;

    //public final static double BEACON_HOME = 0.0;
    //public final static double BEACON_MIN_RANGE  = 0.00;
    //public final static double BEACON_MAX_RANGE  = 1.00;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();
    ColorSensor colorSensor;
    /* Constructor */
    public HardwareDefinition() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontWheel      = hwMap.dcMotor.get("Front Left Wheel");
        leftBackWheel       = hwMap.dcMotor.get("Back Left Wheel");
        rightFrontWheel     = hwMap.dcMotor.get("Front Right Wheel");
        rightBackWheel      = hwMap.dcMotor.get("Back Right Wheel");

        liftLeft            = hwMap.dcMotor.get("Lift Left");
        liftRight           = hwMap.dcMotor.get("Lift Right");


       // sweeper             = hwMap.dcMotor.get("sweeper");

        leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        leftBackWheel.setDirection(DcMotor.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        rightBackWheel.setDirection(DcMotor.Direction.REVERSE);

        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.REVERSE);


      //  sweeper.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        rightBackWheel.setPower(0);

        liftLeft.setPower(0);
        liftRight.setPower(0);
       // sweeper.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        beacon = hwMap.servo.get("Beacon");
        dropLiftLeft        = hwMap.servo.get("Drop Lift Left");
        dropLiftRight        = hwMap.servo.get("Drop Lift Right");
//        servo1 = hwMap.servo.get("Servo1");
  //      servo2 = hwMap.servo.get("Servo2");
    //    servo3 = hwMap.servo.get("Servo3");
          //beacon.setPosition(0.6);
//        servo1.setPosition(BEACON_HOME);
  //      servo2.setPosition(BEACON_HOME;
    //    servo3.setPosition(BEACON_HOME);



    }

    /*
      waitForTick implements a periodic delay. However, this acts like a metronome with a regular
      periodic tick.  This is used to compensate for varying processing times for each cycle.
      The function looks at the elapsed cycle time, and sleeps for the remaining time interval.

      @param periodMs  Length of wait cycle in mSec.
      @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
