package com.clj.blesample.operation;

import android.annotation.TargetApi;
import android.os.Build;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;

import com.clj.blesample.R;
import com.clj.fastble.BleManager;
import com.clj.fastble.callback.BleNotifyCallback;
import com.clj.fastble.callback.BleWriteCallback;
import com.clj.fastble.data.BleDevice;
import com.clj.fastble.exception.BleException;
import com.clj.fastble.utils.HexUtil;

import java.io.UnsupportedEncodingException;

@TargetApi(Build.VERSION_CODES.JELLY_BEAN_MR2)
public class MotionFragment extends Fragment {

    TextView posTextView;
    Button stopButton;
    Button resumeButton;
    Button saveButton;
    SeekBar seekBar1;
    SeekBar seekBar2;
    SeekBar seekBar3;
    SeekBar seekBar4;
    SeekBar seekBar5;
    SeekBar seekBar6;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.motion_layout, null);
        initView(v);
        return v;
    }

    @Override
    public void onResume() {
        super.onResume();




    }
    private void initView(View v) {
        posTextView = v.findViewById(R.id.posTextView);

        stopButton = (Button) v.findViewById(R.id.stop);
        stopButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                stop();
            }
        });

        resumeButton = (Button) v.findViewById(R.id.resume);
        resumeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                resume();
            }
        });

        seekBar1 = v.findViewById(R.id.seekBar1);
        seekBar2 = v.findViewById(R.id.seekBar2);
        seekBar3 = v.findViewById(R.id.seekBar3);
        seekBar4 = v.findViewById(R.id.seekBar4);
        seekBar5 = v.findViewById(R.id.seekBar5);
        seekBar6 = v.findViewById(R.id.seekBar6);

        saveButton = (Button) v.findViewById(R.id.save_button);
        saveButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
               saveData();
            }
        });
    }

    private void saveData() {

        String data = String.valueOf(seekBar1.getProgress()) + ",";
        data += String.valueOf(seekBar2.getProgress()) + ",";
        data += String.valueOf(seekBar3.getProgress()) + ",";
        data += String.valueOf(seekBar4.getProgress()) + ",";
        data += String.valueOf(seekBar5.getProgress()) + ",";
        data += String.valueOf(seekBar6.getProgress());

        byte[] byteData ={};
        try {
             byteData = data.getBytes("UTF-8");
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
        BleDevice bleDevice = ((MotionActivity) getActivity()).getBleDevice();
        BleManager.getInstance().write(
                bleDevice,
                "4fafc201-1fb5-459e-8fcc-c5c9c331914b",
                "aeb5483e-36e1-4688-b7f5-ea07361b26a9",
                byteData,
                new BleWriteCallback() {

                    @Override
                    public void onWriteSuccess(final int current, final int total, final byte[] justWrite) {
                        String g="";
                    }

                    @Override
                    public void onWriteFailure(final BleException exception) {
                        String g="";
                    }
                }
        );


    }

    private void resume() {
        BleDevice bleDevice = ((MotionActivity) getActivity()).getBleDevice();
        BleManager.getInstance().write(
                bleDevice,
                "4fafc201-1fb5-459e-8fcc-c5c9c331914b",
                "beb5483e-36e1-4688-b7f5-ea07361b26a8",
                HexUtil.hexStringToBytes("31"),
                new BleWriteCallback() {

                    @Override
                    public void onWriteSuccess(final int current, final int total, final byte[] justWrite) {
                        String g="";
                    }

                    @Override
                    public void onWriteFailure(final BleException exception) {
                        String g="";
                    }
                }
        );
    }

    private void stop() {
        BleDevice bleDevice = ((MotionActivity) getActivity()).getBleDevice();
        BleManager.getInstance().write(
                bleDevice,
                "4fafc201-1fb5-459e-8fcc-c5c9c331914b",
                "beb5483e-36e1-4688-b7f5-ea07361b26a8",
                HexUtil.hexStringToBytes("30"),
                new BleWriteCallback() {

                    @Override
                    public void onWriteSuccess(final int current, final int total, final byte[] justWrite) {
                        String g="";
                    }

                    @Override
                    public void onWriteFailure(final BleException exception) {
                        String g="";
                    }
                });

    }
}

