package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Dragonoids on 2/27/2017.
 */
@Autonomous(name="Blue Autonomous Button", group="Blue")
public class DragonoidsAutoBlue1 extends DragonoidsAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while (opModeIsActive()) {
            forward(-1.5,.6);
            strafe(1.25, .8);
            forward(-2.2,1);
            adjustHeading();
            adjustRange();
            alignLine(true);
            if (detectColor()==2) {
                forward(-.05,.5);
                sleep(250);
                strafe(.75, 1);
            } else if (detectColor()==1){
                forward(.15, .4);
                sleep(250);
                strafe(.75, 1);
            }
            adjustRange();
            forward(1.63,.65);
            adjustRange();
            adjustHeading();
            alignLine(false);
            if (detectColor()==2) {
                forward(-.05,.5);
                sleep(250);
                strafe(.75, 1);
            } else if (detectColor()==1){
                forward(.15, .4);
                sleep(250);
                strafe(.75, 1);
            }
            strafe(-.9,.8);
            turn(100);
            shoot();
            strafe(.65,1);
            forward(-1.45,.8);

            break;
        }
    }
}
