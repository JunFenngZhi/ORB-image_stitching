#pragma once
#include<iostream>
#include<stdio.h>
#include<opencv2/opencv.hpp>
#include <math.h>
#include<time.h>

using namespace cv;
using namespace std;

//ORB特征
typedef struct Fast_keypoint
{
	int x;					//点的横坐标   原点在左上角
	int y;					//点的纵坐标
	int real_x = 0;			//映射回初始图像的坐标x
	int real_y = 0;			//映射回初始图像的坐标y
	int Fast_response = 0;	//该点的FAST响应值
	int scale_level = 0;		//该点所在金字塔层数
	float Harris_response = 0;	//Harris响应值
	unsigned char Descriptor[32] = { 0 };  //BRIEF描述子（256bits）
	int Get_brief_bit(int index)		   //获取描述子中特定的一位
	{
		unsigned char n = Descriptor[(index - 1) / 8];
		unsigned char a = 1;
		a = a << (index - 1) % 8;
		n = n & a;
		if (n == a)
			return 1;
		return 0;
	}
}Fast_Keypoint;

//存储配对后的点信息
typedef struct Match
{
	Point2f base_point;		//基准图中的点坐标
	Point2f corresponding_point;  //目标图中的点坐标
	int base_index;			//点在基准点集中的下标
	int corresponding_index;//点在目标点集中的下标
}Match_pairs;

class ORB_extractor
{
public:
	ORB_extractor(int nFeatures, float scaleFactor, int nlevels);
	~ORB_extractor();
	void Get_features(Mat grey_image, vector<Fast_Keypoint>&FAST_Keypoints);		//提取ORB特征

private:
	int nFeatures;				//提取的特征点总数	
	int nlevels;				//金字塔层数
	float scaleFactor;			//尺度因子
	vector<Mat>scale_pyramid;   //尺度金字塔
	void Build_pyramid(Mat image);		//建立尺度金字塔
	void Compute_Keypoints(Mat image, int nPoints, int level, vector<Fast_Keypoint>&List);		//对输入图像提取指定数量特征点
	bool FAST16_9(Mat image, vector<Fast_Keypoint>&List, int level, int nPoints, int threshold);	//检测FAST16-9特征点并做非极大值抑制，返回前2N个特征点
	void Compute_Harris(Mat image, vector<Fast_Keypoint>&List);		//对每个特征点计算其harris特征值
	void Compute_BRIEF(vector<Fast_Keypoint>&List);		//计算描述子  

};

class LSH_match
{
public:
	LSH_match(int K, int L, float threshold);
	~LSH_match();
	void Create_HashLists(vector<Fast_Keypoint>&ORB_Features);	//将基准数据放入hash表中(构建hash表)
	void BF_match(vector<Fast_Keypoint>&ORB_Features, vector<Fast_Keypoint>&Base_Features, vector<Match_pairs>& match_result);//计算哈希值后，与相应桶内的点暴力匹配
	int ComputeHamming_Distance(unsigned char Descriptor_1[32], unsigned char Descriptor_2[32]);		//计算两个描述子间的汉明距离

private:
	int K;		//哈希键长
	int L;		//哈希表数量
	float threshold;     //匹配阈值
	vector<map<unsigned long, vector<int>>>Hash_lists;	//哈希表集合
	vector<vector<int>>HashFunctions;	//哈希函数集
	void Create_HashFunction();		    //构建哈希函数

};

Mat My_RANSAC(vector<Match_pairs>&Match_List);