package frc.robot.utils;

import java.io.ObjectInputFilter.Status;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class CtreBaseRefreshManager {
    // create a singleton
    private static CtreBaseRefreshManager instance = null;

    // create a arraylist with all of the status signals
    private static ArrayList<StatusSignal> ctreBaseRefreshList = new ArrayList<StatusSignal>();
    // make a regular list that is not an arraylist

    

    public static CtreBaseRefreshManager getInstance() {
        if (instance == null) {
            instance = new CtreBaseRefreshManager();
        }
        return instance;
    }

    private CtreBaseRefreshManager() {
    }

    public void updateAll(){
        // use BaseSignal to update all of the signals
        BaseStatusSignal.refreshAll(ctreBaseRefreshList.stream().toArray(StatusSignal[]::new));
        




    }

    public void addSignals(ArrayList<StatusSignal> signals){
        ctreBaseRefreshList.addAll(signals);

    }


    


}
