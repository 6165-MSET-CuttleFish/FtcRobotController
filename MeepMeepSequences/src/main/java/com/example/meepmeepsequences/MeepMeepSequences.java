package com.example.meepmeepsequences;

import com.example.meepmeepsequences.util.Details;
import com.example.meepmeepsequences.util.Detector;

public class MeepMeepSequences {
    public static void main(String[] args) {
        Details.location = Detector.Location.RIGHT;
        new BasicCarouselPath().carouselPath(true).start();
    }
}