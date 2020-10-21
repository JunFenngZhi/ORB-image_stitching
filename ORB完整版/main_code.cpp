
#include"ORB_code.h"
#include"stitch_image.h"


Mat  Copy_Pic(Mat image_one, Mat image_two);


void main()
{
	//基准图特征提取
	Mat Base_Pic = imread("E:\\图像拼接——项目\\拼接素材\\material_2\\r1.jpg");
	Mat Base_Pic_Grey;
	vector<Fast_Keypoint>Features_Base;
	cvtColor(Base_Pic, Base_Pic_Grey, CV_BGR2GRAY);
	ORB_extractor Base_Extracor(2000, 1.414, 1);		//基准图的金字塔层数小于实时图，但特征点数量要多
	Base_Extracor.Get_features(Base_Pic_Grey, Features_Base);

	//实时图特征提取
	Mat Pic1 = imread("E:\\图像拼接——项目\\拼接素材\\material_2\\small_2.jpg");
	Mat Pic1_Grey;
	vector<Fast_Keypoint>Features_pic1;
	cvtColor(Pic1, Pic1_Grey, CV_BGR2GRAY);
	ORB_extractor Extracor_Pic1(500, 1.2, 10);
	Extracor_Pic1.Get_features(Pic1_Grey, Features_pic1);

	//匹配
	vector<Match_pairs> match_result;		//存储匹配结果
	LSH_match Matching(16, 3, 0.5);
	Matching.Create_HashLists(Features_Base);
	Matching.BF_match(Features_pic1,Features_Base,match_result);
	cout << "初步匹配点数：" << match_result.size() << endl;
	cout << endl;

	Mat H = My_RANSAC(match_result);
	//Mat stitch_image = Stitch(H, Pic1, Base_Pic);
	//imshow("拼接结果", stitch_image);
	//imwrite("E:\\图像拼接——项目\\拼接素材\\material_2\\r2.jpg",stitch_image);
	
	///////展示效果////////////
	Mat result = Copy_Pic(Base_Pic, Pic1);   //Base在左边（图1），实时图在右边（图2）
	//第一幅图片画点
	for (int i = 0; i < Features_Base.size(); i++)
	{
		cv::Point point;			//特征点，用以画在图像中  
		point.x = Features_Base[i].real_x;	//特征点在图像中横坐标  
		point.y = Features_Base[i].real_y;	//特征点在图像中纵坐标  
		cv::circle(result, point, 1, cv::Scalar(255, 0, 0));	//在图像中画出特征点，1是圆的半径
	}
	//第二幅图片画点
	for (int i = 0; i <Features_pic1 .size(); i++)
	{
		cv::Point point;						//特征点，用以画在图像中  
		point.x = Features_pic1[i].real_x + Base_Pic.cols;	//特征点在图像中横坐标  
		point.y = Features_pic1[i].real_y;				//特征点在图像中纵坐标  
		cv::circle(result, point, 1, cv::Scalar(0, 0, 255));	//在图像中画出特征点，1是圆的半径
	}
	//画线，连接匹配点
	for (int i = 0; i < match_result.size(); i++)
	{
		Point2i des_point=match_result[i].corresponding_point;
		des_point.x += Base_Pic.cols;
		line(result,match_result[i].base_point, des_point, cv::Scalar(0, 255, 0));
	}

	//resize(result, result, Size(result.cols/1.5, result.rows/1.5));
	imshow("匹配结果", result);
	waitKey(0);

	/*
	Mat Base_Pic = imread("E:\\图像拼接——项目\\拼接素材\\material_2\\small_3.jpg");
	resize(Base_Pic, Base_Pic,Size(600,600));
	imwrite("E:\\图像拼接——项目\\拼接素材\\material_2\\small_3.jpg", Base_Pic);
	*/
	
}

Mat  Copy_Pic(Mat image_one, Mat image_two)//不要求两图片等大
{
	int Height = 0;
	if (image_one.rows >= image_two.rows)  //根据最大尺寸确定大小
		Height = image_one.rows;
	else
		Height = image_two.rows;

	Mat result = Mat::zeros(Height, image_one.cols + image_two.cols + 1, CV_8UC3);
	Mat Roi = result(Rect(0, 0, image_one.cols, image_one.rows));   //Rect( _x, _y, _Tp _width, _Tp _height)，定义一个左上角点坐标为(_x, _y)的_width*_height矩形窗口！
	image_one.copyTo(Roi);   //复制图片1
	Mat Roi1 = result(Rect(image_one.cols + 1, 0, image_two.cols, image_two.rows));
	image_two.copyTo(Roi1);  //复制图片2

	return result;

}
