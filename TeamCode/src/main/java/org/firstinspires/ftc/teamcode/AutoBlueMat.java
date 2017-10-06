package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Dragonoids on 2/27/2017.
 */
@Autonomous(name="Blue Auto Mat", group="Blue")
public class AutoBlueMat extends DragonoidsAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while (opModeIsActive()) {
            deployColor(true);
            sleep(3000);
            detectColor();
            sleep(500);
            deployColor(false);
            break;
        }
    }
}

