package com.scanit.likewise.scanitandroid;

import android.content.Intent;
import android.graphics.Bitmap;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.ImageView;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

public class ImageShow extends AppCompatActivity {
    ImageView iv;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_image_show);
        Intent intent = getIntent();
        long addr = intent.getLongExtra("imgAddr", 0);
        Mat tempImg = new Mat( addr );
        Mat img = tempImg.clone();
        Bitmap bm = Bitmap.createBitmap(img.cols(), img.rows(),Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(img, bm);

        // find the imageview and draw it!
        iv = (ImageView) findViewById(R.id.extracted_iv);
        iv.setImageBitmap(bm);

    }
}
