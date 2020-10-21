#include"stitch_image.h"


int  Check_pixel(int y0, int x0, Point2i Pic1_vertex[], int min_x, int min_y, const Mat& Pic2)
{
	int flag = 0;
	Point2i A = Pic1_vertex[0];
	Point2i B = Pic1_vertex[1];
	Point2i C = Pic1_vertex[3];
	Point2i D = Pic1_vertex[2];

	if (((D.x - A.x)*(y0 - A.y) - (D.y - A.y)*(x0 - A.x)) < 0 && ((B.x - A.x)*(y0 - A.y) - (B.y - A.y)*(x0 - A.x)) > 0 &&
		((D.x - C.x)*(y0 - C.y) - (D.y - C.y)*(x0 - C.x)) > 0 && ((C.x - B.x)*(y0 - B.y) - (C.y - B.y)*(x0 - B.x)) > 0)
	{
		flag = flag + 1;  //满足四个边界的约束，确定该像素在Pic1投影面内
	}

	if (x0 >= abs(min_x) && y0 >= abs(min_y) && x0 <= (Pic2.cols - 1 + abs(min_x)) && y0 <= (Pic2.rows - 1 + abs(min_y)))
	{
		flag = flag + 2;  //该像素在Base_Pic的区域中
	}

	return flag;

}
int  Bilinear_interpolation(unsigned char RGB[], Mat Pic1, int x_d, int y_d, const Mat& H)
{

	float a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
	float x = 0, y = 0;

	a = H.at<float>(2, 0)*x_d - H.at<float>(0, 0);
	b = H.at<float>(2, 1)*x_d - H.at<float>(0, 1);
	c = H.at<float>(0, 2) - x_d;
	d = H.at<float>(2, 0)*y_d - H.at<float>(1, 0);
	e = H.at<float>(2, 1)*y_d - H.at<float>(1, 1);
	f = H.at<float>(1, 2) - y_d;
	x = (b * f - c * e) / (b * d - a * e); //将(x_d,y_d)由投影平面（Base_Pic）反映射回源图（Pic1），确定浮点型坐标(x,y)
	y = (c * d - a * f) / (b * d - a * e);
	float u = x - int(x), v = y - int(y);

	if ((int(x) >= (Pic1.cols - 2)) || (int(y) >= (Pic1.rows - 2)) || (x < 0) || (y < 0)) //放弃边界点的插值
		return 0;

	Point2i A = Point2i(int(x), int(y));
	Point2i B = Point2i(int(x + 1), int(y));
	Point2i C = Point2i(int(x), int(y + 1));
	Point2i D = Point2i(int(x + 1), int(y + 1));

	for (int i = 0; i < 3; i++)
	{
		RGB[i] = (1 - u)*(1 - v)*(Pic1.at<Vec3b>(A.y, A.x)[i]) + v * (1 - u)*(Pic1.at<Vec3b>(C.y, C.x)[i])    //对RGB三通道做双线性插值
			+ u * (1 - v)*(Pic1.at<Vec3b>(B.y, B.x)[i]) + v * u*(Pic1.at<Vec3b>(D.y, D.x)[i]);
	}

}
int  Check_distance(Point2i E, Point2i F, int x_d, int y_d, const int Width, float& distance)
{
	//直接使用公式计算点到直线的距离
	float A = F.y - E.y;
	float B = -1 * (F.x - E.x);
	float C = (F.x - E.x)*E.y - (F.y - E.y)*E.x;
	distance = abs((A*x_d + B * y_d + C) / sqrt(A * A + B * B));
	if (distance <= Width)
		return 1;  //点距离直线EF距离小于Width
	else
		return 0;

}
Mat Stitch(const Mat& H, const Mat& Pic1, const Mat& Base_Pic)
{
	int min_x = 0, min_y = 0, max_x = 0, max_y = 0;			 //记录Pic1 坐标转换后的坐标极值
	Point2i Pic1_vertex[4] = { Point2i(0,0),Point2i(Pic1.cols - 1,0),Point2i(0,Pic1.rows - 1),Point2i(Pic1.cols - 1,Pic1.rows - 1) };//Pic1的四个顶点坐标(x,y),初始时为未投影坐标

	//使用Pic1四个顶点计算投影坐标极值，极值用于Base_Pic平面上的坐标平移
	for (int i = 0; i < 4; i++)
	{
		int predict_x = (H.at<float>(0, 0)* Pic1_vertex[i].x + H.at<float>(0, 1)* Pic1_vertex[i].y + H.at<float>(0, 2)) /
			(H.at<float>(2, 0)* Pic1_vertex[i].x + H.at<float>(2, 1)* Pic1_vertex[i].y + H.at<float>(2, 2));
		int predict_y = (H.at<float>(1, 0)* Pic1_vertex[i].x + H.at<float>(1, 1)* Pic1_vertex[i].y + H.at<float>(1, 2)) /
			(H.at<float>(2, 0)* Pic1_vertex[i].x + H.at<float>(2, 1)* Pic1_vertex[i].y + H.at<float>(2, 2));
		Pic1_vertex[i].x = predict_x;
		Pic1_vertex[i].y = predict_y;   //存储Pic1投影后的坐标
		cout << "Pic1 映射后四个坐标: (" << predict_y << "," << predict_x << " )" << endl;

		if (predict_x > max_x)
			max_x = predict_x;
		if (predict_y > max_y)
			max_y = predict_y;
		if (predict_x < min_x)
			min_x = predict_x;
		if (predict_y < min_y)
			min_y = predict_y;
	}
	cout << "映射后坐标极值：" << endl;
	cout << "min_x:" << min_x << " min_y:" << min_y << endl;
	cout << "max_x:" << max_x << " max_y:" << max_y << endl;
	Mat Transformed_image = Mat::zeros(max_y + abs(min_y), Base_Pic.cols + abs(min_x) + 5, CV_8UC3);  //转换后的目标图（Base_Pic'基准）

	//平移Base_Pic坐标系(平移Pic1图4个顶点映射到Base_Pic上的坐标，后面对应仍要平移Base_Pic原图)至Base_Pic '坐标，让Pic1投影区域能够全部显示，解决负坐标的问题
	for (int i = 0; i < 4; i++)
	{
		Pic1_vertex[i].x = Pic1_vertex[i].x + abs(min_x);
		Pic1_vertex[i].y = Pic1_vertex[i].y + abs(min_y);
	}

	//对Base_Pic'坐标系逐个分块（Pic1投影区域\公共区域\Base_Pic区域），确定Base_Pic '坐标下的每个像素的值
	for (int i = 0; i < Transformed_image.rows; i++)
	{
		for (int j = 0; j < Transformed_image.cols; j++)
		{
			int temp = Check_pixel(i, j, Pic1_vertex, min_x, min_y, Base_Pic);
			if (temp == 1)	//只在Pic1的投影点
			{
				unsigned char RGB[3] = { 0,0,0 };
				Bilinear_interpolation(RGB, Pic1, j - abs(min_x), i - abs(min_y), H);  //双线性插值获取像素值（注意（j,i）坐标应先恢复为未平移的值！）
				Transformed_image.at<Vec3b>(i, j)[0] = RGB[0];
				Transformed_image.at<Vec3b>(i, j)[1] = RGB[1];
				Transformed_image.at<Vec3b>(i, j)[2] = RGB[2];
			}
			else if (temp == 2) //只在Base_Pic上的点
			{
				Transformed_image.at<Vec3b>(i, j)[0] = Base_Pic.at<Vec3b>(i - abs(min_y), j - abs(min_x))[0]; //RGB对应赋值,坐标减去平移量，对应回未平移时的Base_Pic坐标
				Transformed_image.at<Vec3b>(i, j)[1] = Base_Pic.at<Vec3b>(i - abs(min_y), j - abs(min_x))[1];
				Transformed_image.at<Vec3b>(i, j)[2] = Base_Pic.at<Vec3b>(i - abs(min_y), j - abs(min_x))[2];
			}
			else if (temp == 3) //公共区域点
			{
				const int Width = abs(max_x) / 3;	//过渡区域宽度
				unsigned char RGB[3] = { 0,0,0 };
				float distance = 0;				//保存像素到pic1边缘的距离  
				Bilinear_interpolation(RGB, Pic1, j - abs(min_x), i - abs(min_y), H);		//双线性插值获取像素值（注意（j,i）坐标应先恢复为未平移的值！）

				if (Check_distance(Pic1_vertex[1], Pic1_vertex[3], j, i, Width, distance))  //像素在过渡区域内（只计算了到重叠区左边缘，考虑别的情况？？）
				{
					float Weight = (1.0 / Width)*distance; //计算权值（渐入渐出法）
					Transformed_image.at<Vec3b>(i, j)[0] = (unsigned char)(Weight * RGB[0] + (1 - Weight)*  Base_Pic.at<Vec3b>(i - abs(min_y), j - abs(min_x))[0]);  //加权平均，消除间隙
					Transformed_image.at<Vec3b>(i, j)[1] = (unsigned char)(Weight * RGB[1] + (1 - Weight)*  Base_Pic.at<Vec3b>(i - abs(min_y), j - abs(min_x))[1]);
					Transformed_image.at<Vec3b>(i, j)[2] = (unsigned char)(Weight * RGB[2] + (1 - Weight)*  Base_Pic.at<Vec3b>(i - abs(min_y), j - abs(min_x))[2]);

				}
				else //像素在过渡区域外
				{
					Transformed_image.at<Vec3b>(i, j)[0] = RGB[0];
					Transformed_image.at<Vec3b>(i, j)[1] = RGB[1];
					Transformed_image.at<Vec3b>(i, j)[2] = RGB[2];
				}
			}

		}
	}

	return Transformed_image;

}