package com.example.meepmeepsequences.util;

public class Robot {
    public static Deposit.State getLevel(Detector.Location location) {
        switch (location) {
            case LEFT: return Deposit.State.IDLE;
            case MIDDLE: return Deposit.State.LEVEL2;
            case RIGHT: return Deposit.State.LEVEL3;
        }
        return Deposit.State.LEVEL3;
    }
}
