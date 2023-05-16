package com.example.visualizer;

import com.example.visualizer.pathing.geometry.SriPoint;
import com.example.visualizer.pathing.path.SriPath;

import java.awt.BasicStroke;
import java.awt.Canvas;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferStrategy;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import javax.swing.JFrame;

public class Visualizer {
    public static void main(String[] args) {
        final String title = "Bezier Visualizer";
        final int width = 1800;
        final int height = width / 16 * 9;

        JFrame frame = new JFrame(title);
        frame.setSize(width, height);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setResizable(false);
        frame.setVisible(true);

        Canvas canvas = new Canvas();
        canvas.setSize(width, height);
        canvas.setBackground(Color.BLACK);
        canvas.setVisible(true);
        canvas.setFocusable(false);

        frame.add(canvas);

        canvas.createBufferStrategy(3);

        boolean running = true;

        BufferStrategy bufferStrategy;
        Graphics2D graphics;

        SriPoint p1 = new SriPoint(500,500);
        SriPoint p2 = new SriPoint(800,500);
        SriPoint p3 = new SriPoint(800,800);
        SriPoint p4 = new SriPoint(1000,800);

        SriPath sriPath = new SriPath(p1,p2,p3,p4,30,20);
        ArrayList<SriPoint> path = sriPath.getBeizerPath();
        int currPoint = 0;
        while (running) {
            bufferStrategy = canvas.getBufferStrategy();
            graphics = (Graphics2D) bufferStrategy.getDrawGraphics().create();
            graphics.clearRect(0, 0, width, height);

            graphics.setStroke(new BasicStroke(1));

            graphics.setColor(Color.GREEN);
            graphics.drawLine( (int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
            graphics.drawLine( (int) p2.x, (int) p2.y, (int) p3.x, (int) p3.y);
            graphics.drawLine( (int) p3.x, (int) p3.y, (int) p4.x, (int) p4.y);


            graphics.setColor(Color.blue);
            for(int i=0; i < path.size() - 1; i++){
                graphics.drawLine((int) path.get(i).x, (int) path.get(i).y,(int) path.get(i+1).x, (int) path.get(i+1).y);
            }

            graphics.setColor(Color.WHITE);
            graphics.drawOval((int) path.get(currPoint).x - 8, (int) path.get(currPoint).y - 8, 50, 50);
            graphics.setColor(Color.RED);

            graphics.setStroke(new BasicStroke(2));
            for(SriPoint s : path) {
                graphics.drawLine((int) s.x, (int) s.y, (int) s.x, (int) s.y);

            }

            bufferStrategy.show();
            graphics.dispose();

            currPoint++;
            if(currPoint == path.size()) running = false;
            try {
                TimeUnit.MILLISECONDS.sleep(10);
            } catch (Exception e) {}
        }
    }

}
