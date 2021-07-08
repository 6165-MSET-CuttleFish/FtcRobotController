package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;

public interface Component {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    void update();
}
