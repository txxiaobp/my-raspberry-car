package com.example.car;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import android.os.AsyncTask;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;

import java.io.IOException;
import java.io.OutputStream;

import java.net.Socket;

import android.widget.Button;
import android.widget.EditText;


public class MainActivity extends AppCompatActivity implements View.OnClickListener, View.OnTouchListener
{
    private String ip;
    private int left = 0;
    private int right = 1;
    private int up = 2;
    private int down = 3;
    private int mouseX = 4;
    private int mouseY = 5;
    private int halt = 6;

    private String direction_str[] = {"0", "0", "0", "0", "0", "0", "0"};


    private Button leftButton;
    private Button rightButton;
    private Button upButton;
    private Button downButton;
    private EditText ipAddress;


    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        leftButton = (Button)findViewById(R.id.Left);
        rightButton = (Button)findViewById(R.id.Right);
        upButton = (Button)findViewById(R.id.Up);
        downButton = (Button)findViewById(R.id.Down);
        ipAddress = (EditText)findViewById(R.id.IpAddress);
        ipAddress.requestFocus();

        leftButton.setOnTouchListener((View.OnTouchListener) this);
        rightButton.setOnTouchListener((View.OnTouchListener) this);
        upButton.setOnTouchListener((View.OnTouchListener) this);
        downButton.setOnTouchListener((View.OnTouchListener) this);


    }

    @Override
    public void onClick(View v) {
        switch (v.getId())
        {
            case R.id.Up:
                upCall();
                break;
            case R.id.Down:
                downCall();
                break;
            case R.id.Left:
                leftCall();
                break;
            case R.id.Right:
                rightCall();
                break;
            case R.id.Connect:
                connect();
                break;
            case R.id.Halt:
                haltCall();
                break;
        }
    }

    private void alertCallBack(String str)
    {
        new AlertDialog.Builder(MainActivity.this)
                .setTitle("alert")
                .setMessage(str)
                .setPositiveButton("Ok", null)
                .show();
    }


    Socket socket = null;
    OutputStream writer = null;

    private String generateStr()
    {
        int length = direction_str.length;
        String retStr = "*";
        for (int i = 0; i < length; i++)
        {
            if (i != length - 1)
            {
                retStr += direction_str[i] + ",";
            }
            else
            {
                retStr +=  direction_str[i] + "&~";
            }
        }
        return retStr;
    }
    public void connect()
    {
        ip = ipAddress.getText().toString();
        ipAddress.clearFocus();

        new AsyncTask<Void, String, Void>() {
            @Override
            protected Void doInBackground(Void... params)
            {
                try
                {
                    socket = new Socket(ip, 10999);
                    writer = socket.getOutputStream();
                    while (true)
                    {
                        String sendStr =  generateStr();
                        send(sendStr);
                        System.out.println(sendStr);
                        resetDirectionStr();
                        try {
                            Thread.sleep(60);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }

                } catch (IOException e) {
                    System.out.println("Failure"); // 链接失败
                    e.printStackTrace();
                }

                return null;
            }

        }.execute();
    }

    public void send(String str)
    {
        while (writer == null);
        try {
            writer.write(str.getBytes());
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            writer.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void resetDirectionStr()
    {
        direction_str[left] = "0";
        direction_str[right] = "0";
        direction_str[up] = "0";
        direction_str[down] = "0";
        direction_str[halt] = "0";
    }

    private void  upCall()
    {
        direction_str[up] = "1";
    }

    private void  downCall()
    {
        direction_str[down] = "1";
    }

    private void  leftCall()
    {
        direction_str[left] = "1";
    }

    private void  rightCall()
    {
        direction_str[right] = "1";
    }

    private void  haltCall()
    {
        direction_str[halt] = "1";
    }

    private int upButtonDown;
    private int downButtonDown;
    private int leftButtonDown;
    private int rightButtonDown;

    @Override
    public boolean onTouch(View v, MotionEvent event)
    {
        System.out.println("ontouch");
        switch (v.getId())
        {
            case R.id.Up:
                if(event.getAction() == MotionEvent.ACTION_DOWN)
                {
                    upButtonDown = 1;
                    new Thread(new Runnable() {
                        @Override
                        public void run()
                        {
                            while (upButtonDown == 1)
                            {
                                upCall();
                            }
                        }
                    }).start();
                }
                else if (event.getAction() == MotionEvent.ACTION_UP)
                {
                    upButtonDown = 0;
                }
                break;
            case R.id.Down:
                if(event.getAction() == MotionEvent.ACTION_DOWN)
                {
                    downButtonDown = 1;
                    new Thread(new Runnable() {
                        @Override
                        public void run()
                        {
                            while (downButtonDown == 1)
                            {
                                downCall();
                            }
                        }
                    }).start();
                }
                else if (event.getAction() == MotionEvent.ACTION_UP)
                {
                    downButtonDown = 0;
                }
                break;
            case R.id.Left:
                if(event.getAction() == MotionEvent.ACTION_DOWN)
                {
                    leftButtonDown = 1;
                    new Thread(new Runnable() {
                        @Override
                        public void run()
                        {
                            while (leftButtonDown == 1)
                            {
                                leftCall();
                            }
                        }
                    }).start();
                }
                else if (event.getAction() == MotionEvent.ACTION_UP)
                {
                    leftButtonDown = 0;
                }
                break;
            case R.id.Right:
                if(event.getAction() == MotionEvent.ACTION_DOWN)
                {
                    rightButtonDown = 1;
                    new Thread(new Runnable() {
                        @Override
                        public void run()
                        {
                            while (rightButtonDown == 1)
                            {
                                rightCall();
                            }
                        }
                    }).start();
                }
                else if (event.getAction() == MotionEvent.ACTION_UP)
                {
                    rightButtonDown = 0;
                }
                break;
        }
        return true;
    }
}