#include"ORB_code.h"

static int bit_pattern_31[256 * 4] =
{
	8,-3, 9,5/*mean (0), correlation (0)*/,
	4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
	-11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
	7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
	2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
	1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
	-2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
	-13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
	-13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
	10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
	-13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
	-11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
	7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
	-4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
	-13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
	-9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
	12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
	-3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
	-6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
	11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
	4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
	5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
	3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
	-8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
	-2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
	-13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
	-7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
	-4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
	-10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
	5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
	5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
	1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
	9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
	4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
	2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
	-4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
	-8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
	4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
	0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
	-13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
	-3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
	-6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
	8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
	0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
	7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
	-13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
	10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
	-6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
	10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
	-13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
	-13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
	3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
	5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
	-1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
	3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
	2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
	-13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
	-13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
	-13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
	-7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
	6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
	-9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
	-2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
	-12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
	3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
	-7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
	-3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
	2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
	-11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
	-1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
	5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
	-4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
	-9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
	-12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
	10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
	7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
	-7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
	-4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
	7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
	-7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
	-13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
	-3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
	7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
	-13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
	1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
	2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
	-4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
	-1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
	7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
	1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
	9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
	-1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
	-13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
	7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
	12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
	6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
	5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
	2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
	3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
	2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
	9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
	-8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
	-11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
	1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
	6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
	2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
	6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
	3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
	7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
	-11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
	-10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
	-5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
	-10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
	8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
	4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
	-10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
	4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
	-2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
	-5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
	7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
	-9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
	-5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
	8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
	-9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
	1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
	7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
	-2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
	11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
	-12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
	3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
	5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
	0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
	-9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
	0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
	-1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
	5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
	3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
	-13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
	-5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
	-4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
	6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
	-7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
	-13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
	1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
	4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
	-2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
	2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
	-2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
	4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
	-6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
	-3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
	7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
	4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
	-13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
	7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
	7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
	-7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
	-8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
	-13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
	2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
	10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
	-6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
	8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
	2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
	-11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
	-12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
	-11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
	5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
	-2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
	-1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
	-13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
	-10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
	-3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
	2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
	-9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
	-4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
	-4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
	-6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
	6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
	-13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
	11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
	7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
	-1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
	-4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
	-7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
	-13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
	-7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
	-8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
	-5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
	-13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
	1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
	1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
	9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
	5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
	-1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
	-9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
	-1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
	-13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
	8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
	2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
	7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
	-10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
	-10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
	4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
	3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
	-4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
	5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
	4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
	-9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
	0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
	-12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
	3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
	-10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
	8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
	-8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
	2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
	10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
	6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
	-7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
	-3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
	-1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
	-3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
	-8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
	4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
	2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
	6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
	3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
	11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
	-3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
	4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
	2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
	-10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
	-13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
	-13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
	6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
	0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
	-13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
	-9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
	-13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
	5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
	2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
	-1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
	9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
	11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
	3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
	-1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
	3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
	-13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
	5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
	8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
	7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
	-10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
	7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
	9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
	7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
	-1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};

bool Compare_FAST(Fast_Keypoint a, Fast_Keypoint b)
{
	return a.Fast_response > b.Fast_response;
}
bool Compare_Harris(Fast_Keypoint a, Fast_Keypoint b)
{
	return a.Harris_response > b.Harris_response;
}
void Show_points(vector<Fast_Keypoint>&Point_list, Mat image)
{
	for (int i = 0; i < Point_list.size(); i++)
	{
		cv::Point point;			//特征点，用以画在图像中  
		point.x = Point_list[i].real_x;	//特征点在图像中横坐标  
		point.y = Point_list[i].real_y;	//特征点在图像中纵坐标  
		cv::circle(image, point, 2 + Point_list[i].scale_level * 3, cv::Scalar(255, 255, 255));	//在图像中画出特征点，1是圆的半径   尺度大，圆大
	}
	imshow("result", image);
	waitKey(0);

}

///////////ORB特征提取///////////////
ORB_extractor::ORB_extractor(int nFeatures, float scaleFactor, int nlevels)
{
	this->nFeatures = nFeatures;
	this->nlevels = nlevels;
	this->scaleFactor = scaleFactor;
}
ORB_extractor::~ORB_extractor()
{
}
void ORB_extractor::Build_pyramid(Mat grey_image)
{
	Mat scale_image = grey_image.clone();   //Mat对象传值调用，函数内对其的操作将会影响原对象！
	scale_pyramid.push_back(scale_image);   //第0层
	for (int i = 1; i < nlevels; i++)
	{
		int Height = cvRound(scale_image.rows / scaleFactor);
		int Width = cvRound(scale_image.cols / scaleFactor);
		resize(scale_image, scale_image, Size(Width, Height));
		scale_pyramid.push_back(scale_image);
	}
	/*
	for (int i = 0; i < nlevels; i++)
	{
		imshow("test", scale_pyramid[i]);
		waitKey(0);
	}
	*/
}
bool ORB_extractor::FAST16_9(Mat image, vector<Fast_Keypoint>&List, int level, int nPoints, int threshold)
{
	int row[16] = { -3,-3,-2,-1,0,1,2,3,3,3,2,1,0,-1,-2,-3 };	 //待检查点与中心点的行坐标偏差（顺序1-16）
	int col[16] = { 0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1 };	 //待检查点与中心点的列坐标偏差
	Mat table = Mat::zeros(image.rows, image.cols, CV_32SC1);	 //特征点位置记录表
	vector<Point2i>point_candidata_1;						//初步特征点(只存坐标)（未极大值抑制）
	vector<Fast_Keypoint>point_candidata_2;						 //极大值抑制后的特征点
	int Edge_threshold = 15;			//FAST中跳过的边缘像素个数

	//FAST检测(简单定义)
	for (int i = Edge_threshold; i < image.rows - Edge_threshold; i++)
	{
		for (int j = Edge_threshold; j < image.cols - Edge_threshold; j++)
		{
			int delta = 0;		//差值
			int count = 0;		//计数变量
			int sum = 0;        		//记录总差值

			//快速检测
			delta = abs(image.at<uchar>(i + row[0], j + col[0]) - image.at<uchar>(i, j));
			sum += delta;
			if (delta >= threshold)  //检查1号点
			{
				count++;
			}
			delta = abs(image.at<uchar>(i + row[8], j + col[8]) - image.at<uchar>(i, j));
			sum += delta;
			if (delta >= threshold)  //检查9号点
			{
				count++;
			}
			if (count != 2) //1、9号点都符合要求，才做下一步
				continue;
			delta = abs(image.at<uchar>(i + row[4], j + col[4]) - image.at<uchar>(i, j));
			sum += delta;
			if (delta >= threshold)  //检查5号点
			{
				count++;
			}
			delta = abs(image.at<uchar>(i + row[12], j + col[12]) - image.at<uchar>(i, j));
			sum += delta;
			if (delta >= threshold)  //检查13号点
			{
				count++;
			}
			if (count < 3)  //四个正方向点至少三个满足要求，才考虑为候选点
				continue;

			//具体检测其余点
			for (int k = 0; k < 16; k++)
			{
				if (k == 0 || k == 8 || k == 4 || k == 12)
					continue;
				delta = abs(image.at<uchar>(i + row[k], j + col[k]) - image.at<uchar>(i, j));
				sum += delta;
				if (delta >= threshold)
				{
					count++;
				}
			}

			//初步符合要求，记录下来
			if (count >= 9)
			{
				Point2i temp;
				temp.x = j;
				temp.y = i;
				point_candidata_1.push_back(temp);
				table.at<int>(i, j) = sum;

			}

		}
	}
	//cout << "原始个数" << point_candidata_1.size() << endl;

	//非极大值抑制
	for (int k = 0; k < point_candidata_1.size(); k++)
	{
		//获取待检测点信息
		int x = point_candidata_1[k].x;
		int y = point_candidata_1[k].y;
		int center_response = table.at<int>(y, x);
		int flag = 0;   // 标志待检测点是否为极大(1为非极大，0为极大)

		if (table.at<int>(y, x) == 0)  //该点非极大，去除
			continue;

		for (int i = -2; i <= 2; i++)
		{
			if (flag == 1)
				break;
			for (int j = -2; j <= 2; j++)
			{
				if (center_response >= table.at<int>(i + y, j + x))
				{
					table.at<int>(i + y, j + x) = 0;  //中心比别人大，将别人置零
				}
				else
				{
					table.at<int>(y, x) = 0;  //中心比别人小，自己归零.退出
					flag = 1;
					break;
				}

			}
		}

		if (flag == 0)   //确定为极大值点
		{
			Fast_Keypoint temp;
			temp.x = x;
			temp.y = y;
			temp.Fast_response = center_response;
			temp.scale_level = level;
			point_candidata_2.push_back(temp);
		}

	}
	//cout << "抑制后个数" << point_candidata_2.size() << endl;

	//不足2N个特征点 退出
	if (point_candidata_2.size() < nPoints)
		return false;

	//排序后取FAST响应值最大的前2N个  
	sort(point_candidata_2.begin(), point_candidata_2.end(), Compare_FAST);
	List.insert(List.end(), point_candidata_2.begin(), point_candidata_2.begin() + nPoints);

	return true;

}
void ORB_extractor::Compute_Harris(Mat image, vector<Fast_Keypoint>&List)
{
	int Gauss_model[5][5] = { {1 ,2, 3, 2, 1},
							  {2, 5, 6, 5, 2 },
							  {3, 6, 8, 6, 3},
							  {2, 5, 6, 5, 2},
							  {1, 2, 3, 2, 1} };
	float M[2][2] = { 0 };

	for (int k = 0; k < List.size(); k++)
	{
		int x = List[k].x;
		int y = List[k].y;

		//每个像素点 计算其5*5高斯窗内的M矩阵加权平均
		for (int i = -2; i <= 2; i++)
		{
			for (int j = -2; j <= 2; j++)
			{
				float Ix = (image.at<uchar>(y + i, x + j + 1) - image.at<uchar>(y + i, x + j - 1)) / 2.0;
				float Iy = (image.at<uchar>(y + i + 1, x + j) - image.at<uchar>(y + i - 1, x + j)) / 2.0;
				M[0][0] = M[0][0] + Ix * Ix * Gauss_model[i][j];
				M[1][1] = M[1][1] + Iy * Iy * Gauss_model[i][j];
				M[0][1] = M[0][1] + Ix * Iy * Gauss_model[i][j];
				M[1][0] = M[1][0] + Ix * Iy * Gauss_model[i][j];
			}
		}

		M[0][0] = M[0][0] / 84;
		M[1][0] = M[1][0] / 84;
		M[0][1] = M[0][1] / 84;
		M[1][1] = M[1][1] / 84;
		List[k].Harris_response = (M[0][0] * M[1][1] - M[1][0] * M[1][0]) - 0.04*(M[0][0] + M[1][1]);  //Harris 响应值
	}

}
void ORB_extractor::Compute_Keypoints(Mat image, int nPoints, int level, vector<Fast_Keypoint>&List)
{
	//初步提取FAST 做非极大值抑制后，挑选前2N 个，对2N 个计算harris 取前N个
	int ini_threshold = 20;		//FAST初始阈值
	int min_threshold = 7;		//FAST最小阈值
	vector<Fast_Keypoint>keypoint_candidate;	//候选的2N个特征点

	bool flag = FAST16_9(image, keypoint_candidate, level, 2 * nPoints, ini_threshold);  //使用初始阈值检测
	if (flag == false)		//特征点数量不足2N
	{
		keypoint_candidate.clear();
		flag = FAST16_9(image, keypoint_candidate, level, 2 * nPoints, min_threshold);  //使用最小阈值重新检测   
		if (flag == false)
		{
			cout << "仍然不足2N个，考虑降低阈值或减少特征点数量" << endl;
			system("pause");
			exit(0);
		}
	}

	Compute_Harris(image, keypoint_candidate);  //计算Harris响应值  
	sort(keypoint_candidate.begin(), keypoint_candidate.end(), Compare_Harris);
	for (int k = 0; k < nPoints; k++)
	{
		int x = keypoint_candidate[k].x;
		int y = keypoint_candidate[k].y;
		keypoint_candidate[k].real_x = cvRound(x * pow(scaleFactor, level));   //映射回原图坐标
		keypoint_candidate[k].real_y = cvRound(y * pow(scaleFactor, level));
	}
	List.insert(List.end(), keypoint_candidate.begin(), keypoint_candidate.begin() + nPoints);   //排序后挑选最大的前N个

}
void ORB_extractor::Compute_BRIEF(vector<Fast_Keypoint>&List)
{
	for (int i = 0; i < nlevels; i++)
	{
		GaussianBlur(scale_pyramid[i], scale_pyramid[i], Size(7, 7), 2, 2, BORDER_REFLECT_101);
	}
	for (int k = 0; k < List.size(); k++)
	{
		int x = List[k].x;
		int y = List[k].y;
		int level = List[k].scale_level;
		for (int m = 0; m < 32; m++)
		{
			unsigned char val = 0;
			int t0, t1;
			for (int n = 0; n < 8; n++)
			{
				//获取点对像素值t0,t1
				t0 = scale_pyramid[level].at<uchar>(y + bit_pattern_31[m * 32 + 4 * n + 0], x + bit_pattern_31[m * 32 + 4 * n + 1]);
				t1 = scale_pyramid[level].at<uchar>(y + bit_pattern_31[m * 32 + 4 * n + 2], x + bit_pattern_31[m * 32 + 4 * n + 3]);
				val |= (t0 < t1) << n; //将结果存进每一位
				//cout <<n<<":"<<(t0 < t1) << " ";
			}
			//cout << endl;
			List[k].Descriptor[m] = val;
		}

	}
}
void ORB_extractor::Get_features(Mat grey_image, vector<Fast_Keypoint>&FAST_Keypoints)
{
	clock_t startTime, endTime;
	startTime = clock();			//计时开始

	Build_pyramid(grey_image);				//建立尺度金字塔

	//计算每层图像应该提取的特征点数量
	float factor = 1.0f / scaleFactor;
	vector<int>n_perLevels(nlevels);		//存储每层图像应该提取的特征点数
	float n_First = nFeatures * (1 - factor) / (1 - pow(factor, nlevels));	//等比序列求和法则，得到第一层图像的目标关键点数目
	for (int i = 0; i < nlevels; i++)
		n_perLevels[i] = cvRound(n_First * pow(factor, i));

	//提取每层图像的特征点
	for (int i = 0; i < nlevels; i++)
		Compute_Keypoints(scale_pyramid[i], n_perLevels[i], i, FAST_Keypoints);

	Compute_BRIEF(FAST_Keypoints);		//计算BRIEF描述子
	endTime = clock();				//计时结束

	cout << "特征点及描述子计算时间：" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	//Show_points(FAST_Keypoints, grey_image); 

}

///////////LSH快速匹配///////////////
LSH_match::LSH_match(int K, int L, float threshold)
{
	this->K = K;
	this->L = L;
	this->threshold = threshold;
	Create_HashFunction();
}
LSH_match::~LSH_match()
{
}
void LSH_match::Create_HashFunction()
{
	int index = 0;
	for (int i = 0; i < L; i++)
	{
		vector<int>temp;
		for (int j = 0; j < K; j++)
		{
			index = (rand() % (256 - 1 + 1)) + 1;   // 生成[1,256]之间的整数
			temp.push_back(index);
		}
		HashFunctions.push_back(temp);
	}
}
void LSH_match::Create_HashLists(vector<Fast_Keypoint>&ORB_Features)
{
	for (int i = 0; i < L; i++)
	{
		map<unsigned long, vector<int>>temp_lists;
		for (int m = 0; m < ORB_Features.size(); m++)
		{
			unsigned long value = 0;  //哈希值
			for (int k = 0; k < K; k++)
			{
				value = value * 2 + ORB_Features[m].Get_brief_bit(HashFunctions[i][k]);
			}
			temp_lists[value].push_back(m);  //将本点(下标)放入对应的哈希桶中
		}
		Hash_lists.push_back(temp_lists);
	}
}
int LSH_match::ComputeHamming_Distance(unsigned char Descriptor_1[32], unsigned char Descriptor_2[32])
{
	unsigned char result[32] = { 0 };
	int distance = 0;
	for (int i = 0; i < 32; i++)
	{
		result[i] = Descriptor_1[i] ^ Descriptor_2[i]; //对应位置做异或
		distance = distance + __popcnt16(result[i]);
	}
	return distance;

}
void LSH_match::BF_match(vector<Fast_Keypoint>&ORB_Features, vector<Fast_Keypoint>&Base_Features, vector<Match_pairs>&match_result)
{
	clock_t startTime, endTime;
	startTime = clock();			//计时开始

	vector<int>base_index;
	for (int m = 0; m < ORB_Features.size(); m++)
	{
		//将各个哈希表中符合哈希值的桶集合起来
		base_index.clear();
		for (int l = 0; l < L; l++)
		{
			unsigned long key = 0;
			for (int k = 0; k < K; k++)
			{
				key = key * 2 + ORB_Features[m].Get_brief_bit(HashFunctions[l][k]);
			}
			if (Hash_lists[l].find(key) == Hash_lists[l].end()) //key在哈希表中不存在
				continue;
			base_index.insert(base_index.end(), Hash_lists[l][key].begin(), Hash_lists[l][key].end());
		}

		//做暴力匹配
		int min_distance = 256;		//最小汉明距离
		int second_min_distance = 256;	//次小汉明距离
		int min_script = 0;			//最小汉明距离对应的基准点下标(在base_index 中的下标)
		if (base_index.size() == 1)
		{
			min_distance = ComputeHamming_Distance(ORB_Features[m].Descriptor, Base_Features[base_index[0]].Descriptor);  //只有一个点，不用求次小
			if (min_distance < 50)	//满足相似度要求，即认为匹配
			{
				Match_pairs temp;
				temp.base_index = base_index[0];
				temp.corresponding_index = m;
				temp.base_point = Point2f(Base_Features[base_index[0]].real_x, Base_Features[base_index[0]].real_y);
				temp.corresponding_point = Point2f(ORB_Features[m].real_x, ORB_Features[m].real_y);
				match_result.push_back(temp);
			}
			continue;
		}
		for (int k = 0; k < base_index.size(); k++)
		{
			int distance = ComputeHamming_Distance(ORB_Features[m].Descriptor, Base_Features[base_index[k]].Descriptor);
			if (distance > 50)
				continue;
			if (distance < min_distance)  //比最小还要小
			{
				min_distance = distance;
				min_script = k;
			}
			else if (distance < second_min_distance)  //比最小要大但是比次小要小
			{
				second_min_distance = distance;
			}
		}
		if (float(min_distance) / (second_min_distance) < threshold)
		{
			Match_pairs temp;
			temp.base_index = base_index[min_script];
			temp.corresponding_index = m;
			temp.base_point = Point2f(Base_Features[base_index[min_script]].real_x, Base_Features[base_index[min_script]].real_y);
			temp.corresponding_point = Point2f(ORB_Features[m].real_x, ORB_Features[m].real_y);
			match_result.push_back(temp);
		}
	}

	endTime = clock();				//计时结束
	cout << "匹配用时：" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
}

///////////RANSAC算法////////////////
Mat Cal_H(int N[], vector<Point2f>&src_Points, vector<Point2f>&des_Points)
{
	Mat A = Mat::zeros(8, 8, CV_32FC1);  //点对系数矩阵
	Mat B = Mat::zeros(8, 1, CV_32FC1);
	Mat H = Mat::ones(9, 1, CV_32FC1);
	Mat A_inverse;

	for (int i = 0; i < 8; i = i + 2)
	{
		int script = N[i / 2];
		A.at<float>(i, 0) = src_Points[script].x;
		A.at<float>(i, 1) = src_Points[script].y;
		A.at<float>(i, 2) = 1;
		A.at<float>(i, 6) = -1 * src_Points[script].x*des_Points[script].x;
		A.at<float>(i, 7) = -1 * src_Points[script].y*des_Points[script].x;
		A.at<float>(i + 1, 3) = src_Points[script].x;
		A.at<float>(i + 1, 4) = src_Points[script].y;
		A.at<float>(i + 1, 5) = 1;
		A.at<float>(i + 1, 6) = -1 * src_Points[script].x*des_Points[script].y;
		A.at<float>(i + 1, 7) = -1 * src_Points[script].y*des_Points[script].y;
		B.at<float>(i, 0) = des_Points[script].x;
		B.at<float>(i + 1, 0) = des_Points[script].y;
	}
	invert(A, A_inverse);
	Mat X = (A_inverse * B);
	for (int i = 0; i < 8; i++)
	{
		H.at<float>(i, 0) = X.at<float>(i, 0);
	}
	return H.reshape(0, 3); //转为3*3单通道矩阵

}
float Model_error(int i, vector<Point2f>&src_Points, vector<Point2f>&des_Points, Mat& H)
{
	float predict_x = 0, predict_y = 0;
	Point2f point_s = src_Points[i];
	Point2f point_d = des_Points[i];
	float error = 0;

	predict_x = (H.at<float>(0, 0)*point_s.x + H.at<float>(0, 1)*point_s.y + H.at<float>(0, 2)) /
		(H.at<float>(2, 0)*point_s.x + H.at<float>(2, 1)*point_s.y + H.at<float>(2, 2));
	predict_y = (H.at<float>(1, 0)*point_s.x + H.at<float>(1, 1)*point_s.y + H.at<float>(1, 2)) /
		(H.at<float>(2, 0)*point_s.x + H.at<float>(2, 1)*point_s.y + H.at<float>(2, 2));
	error = (predict_x - point_d.x)*(predict_x - point_d.x) + (predict_y - point_d.y)*(predict_y - point_d.y);//预测点和实际点的欧式距离作为评判标准

	return error;


}
int Adjust_niters(float confidence, float inliers_ratio, int origin_niters)
{
	float bainumerator = log(1 - confidence);			//分子部分
	float denominator = log(1 - pow(inliers_ratio, 4));		//分母部分

	int new_niters = cvRound(bainumerator / denominator);		//新的迭代次数
	if (new_niters < origin_niters)
	{
		return new_niters;
	}
	else
		return origin_niters;

}
Mat My_RANSAC(vector<Match_pairs>&Match_List)
{
	int iteration_Times = 10000;			//初始迭代次数
	float threshold = 3;					//错误阀值，小于阀值加入內点
	float confidence = 0.99;				//置信度
	vector<Point2f>src_Points, des_Points;  //基准图点集（correspondence）  实时图点集(base)
	int N[4] = { 0 };						//4个随机抽取的初始内点的下标
	Mat best_H;								//最佳图像变换矩阵
	vector<Match_pairs>Best_match;			//RANSAC筛选后的最佳匹配结果
	int i;

	if (Match_List.size() <= 4)
	{
		cout << "匹配特征点对数太少！" << endl;
		system("pause");
	}
	for (int i = 0; i < Match_List.size(); i++)
	{
		src_Points.push_back(Match_List[i].corresponding_point);
		des_Points.push_back(Match_List[i].base_point);
	}
	for (i = 0; i < iteration_Times; i++)
	{
		vector<Match_pairs>Inliers;						//符合本轮迭代模型的内点对集
		srand(i);										//随机种子设为循环下标i,因为循环速度太快时间种子尚未更新
		for (int k = 0; k < 4; k++)
		{
			N[k] = rand() % Match_List.size();		//随机获取初始假设内点的下标
		}
		Mat H = Cal_H(N, src_Points, des_Points);		//使用初始假设内点计算转换矩阵H
		for (int j = 0; j < Match_List.size(); j++)		//对所有点对进行检验
		{
			if (Model_error(j, src_Points, des_Points, H) <= threshold)	//计算误差小于阈值，这个点对即为内点对
			{
				Inliers.push_back(Match_List[j]);
			}
		}
		if (Inliers.size() > Best_match.size())			//找到内点数更多的模型，认为这个模型更好
		{
			float inliers_ratio = (float)Inliers.size() / Match_List.size();	//当前最佳模型的内点比例
			if (inliers_ratio > 0.9)
				iteration_Times = 0;	//内点比例足够多时，认为模型已经最佳
			else
				iteration_Times = Adjust_niters(confidence,inliers_ratio , iteration_Times);  //调整迭代次数
			Best_match = Inliers;
			best_H = H;
		}
	}
	cout << "RANSAC迭代次数：" << i << endl;
	cout << "RANSAC 筛选后的配对点数：" << Best_match.size() << endl;
	cout << "Homography Mat:" << endl;
	cout << "[ " << best_H.at<float>(0, 0) << ", " << best_H.at<float>(0, 1) << ", " << best_H.at<float>(0, 2) << " ]" << endl;
	cout << "[ " << best_H.at<float>(1, 0) << ", " << best_H.at<float>(1, 1) << ", " << best_H.at<float>(1, 2) << " ]" << endl;
	cout << "[ " << best_H.at<float>(2, 0) << ", " << best_H.at<float>(2, 1) << ", " << best_H.at<float>(2, 2) << " ]" << endl;
	cout << endl;
	Match_List.swap(Best_match);  //将最佳结果返回出去
	return best_H;

}