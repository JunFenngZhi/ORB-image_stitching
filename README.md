# ORB-image_stitch
使用ORB算法进行图像特征点匹配及其描述子计算。根据匹配的特征点计算单应性矩阵，实现图像拼接。  
程序部分参数和实现细节参考了OPENCV源码及ORB算法论文。  

运行环境： VS2017 + OPENCV 3.4.1
## 实验结果
![Image text](https://github.com/JunFenngZhi/ORB-image_stitching/blob/master/results/result1_b.jpg)

## 存在的问题
1、当两张图片视角差异过大或出现较大旋转时，会出现匹配错误。  
2、图像融合部分处理较粗糙，相邻图像拼接处容易产生分层。  
3、ORB特征点检测和匹配速度和论文相比略有差距。
111222
