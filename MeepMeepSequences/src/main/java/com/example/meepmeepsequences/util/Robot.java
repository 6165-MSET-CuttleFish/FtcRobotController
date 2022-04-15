package com.example.meepmeepsequences.util;

public class Robot {
    public static Deposit.Level getLevel(Detector.Location location) {
        switch (location) {
            case LEFT: return Deposit.Level.IDLE;
            case MIDDLE: return Deposit.Level.LEVEL2;
            case RIGHT: return Deposit.Level.LEVEL3;
        }
        return Deposit.Level.LEVEL3;
    }
}
