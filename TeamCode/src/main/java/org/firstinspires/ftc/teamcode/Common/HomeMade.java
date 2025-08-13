package org.firstinspires.ftc.teamcode.Common;

import android.util.Pair;

import java.util.ArrayList;
import java.util.List;

public class HomeMade {
    public List<Pair<Double, Double>> lista;

    public HomeMade()
    {
        lista = new ArrayList<>();
    }
    public void add(double search, double val)
    {
        lista.add(new Pair<>(search, val));
    }
    public double search(double search)
    {
        Pair<Double, Double> pre = new Pair<>(0.d, 0.d);
        Pair<Double, Double> post = lista.get(0);


        for ( int i = 1; i < lista.size(); ++i )
        {
            pre = post;
            post = lista.get(i);

            double dif = (post.second - pre.second) / (post.first - pre.first);

            if ( search < post.first )
                return pre.second +  ( search - pre.first) * dif;
        }

        return 0;
    }
}
