// これは メイン DLL ファイルです。

#include "stdafx.h"

#include "IcPointCloudLibraryProxy.h"
#include "IcPclConst.h"

#include <msclr/marshal_cppstd.h>

using namespace IcPointCloudLibraryProxy;
using namespace IcCalcUtil;
using namespace IcUtil;

// コンストラクタ
IcPclProxyForRoboPath::IcPclProxyForRoboPath()
{

}

//! @brief 点群ファイルから点群データを読み取り
IcPclResult IcPclProxyForRoboPath::ReadPointList(String^ aFilePath, List<IcVector3D^>^ aPointList)
{
	// 点群インスタンス
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	List<IcVector3D^>^ resPointList = gcnew List<IcVector3D^>();

	double expansionRate = 100.0;

	// String ⇒ std::string　の変換
	std::string str = msclr::interop::marshal_as<std::string>(aFilePath);

	// 拡張子
	String^ format = IcUtilCppCli::GetStrAfterKey(aFilePath, ".", false);

	// 拡張子によって処理を変更
	if (format == "pcd") {

		// pcdファイルを読み込む
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(str, *pclPointCloud) == -1) {

			// 読み取り失敗
			Debug::WriteLine("ERROR : ReadPointList()");
			return IcPclResult::IC_PCL_FAILED;

		}

		// 読み取った点群の座標を記録
		for (int i = 0; i < pclPointCloud->size(); i++) {

			IcVector3D^ icPoint = gcnew IcVector3D(
				pclPointCloud->points[i].x,
				pclPointCloud->points[i].y,
				pclPointCloud->points[i].z
			);

			resPointList->Add(icPoint);

		}

	}
	else if (format == "xyz") {

		// xyzファイルを読み込む
		ifstream ifs(str, ios::in);
		if (!ifs) {

			// 読み取り失敗
			Debug::WriteLine("ERROR : ReadPointList()");
			return IcPclResult::IC_PCL_FAILED;

		}
		std::string tmp;
		std::string s;
		// getline()で1行ずつ読み込む
		while (getline(ifs, tmp)) {
			// ここでtmpを煮るなり焼くなりする
			std::cout << "Read" << tmp << std::endl; // そのまま出力
			std::vector<double> pclPoint;
			std::stringstream ss{ tmp };
			while (getline(ss, s, ',')) {     // コンマ（,）で区切って，格納
				double d = std::stod(s);
				pclPoint.push_back(d);
			}

			IcVector3D^ icPoint = gcnew IcVector3D(
				pclPoint[0] * expansionRate,
				pclPoint[1] * expansionRate,
				pclPoint[2] * expansionRate
			);
			
			resPointList->Add(icPoint);

		}
		ifs.close();

	}
	else if (format == "txt") {

		// txtファイルを読み込む
		ifstream ifs(str, ios::in);
		if (!ifs) {

			// 読み取り失敗
			Debug::WriteLine("ERROR : ReadPointList()");
			return IcPclResult::IC_PCL_FAILED;

		}
		std::string tmp;
		std::string s;
		// getline()で1行ずつ読み込む
		while (getline(ifs, tmp)) {
			// ここでtmpを煮るなり焼くなりする
			std::cout << "Read" << tmp << std::endl; // そのまま出力
			std::vector<double> pclPoint;
			std::stringstream ss{ tmp };
			while (getline(ss, s, ' ')) {     // スペース（ ）で区切って，格納
				double d = std::stod(s);
				pclPoint.push_back(d);
			}

			IcVector3D^ icPoint = gcnew IcVector3D(
				pclPoint[0] * expansionRate,
				pclPoint[1] * expansionRate,
				pclPoint[2] * expansionRate
			);

			resPointList->Add(icPoint);

		}
		ifs.close();

	}
	else {

		// 読み取り失敗
		Debug::WriteLine("ERROR : ReadPointList()");
		return IcPclResult::IC_PCL_FAILED;

	}

	// 読み取り結果を格納
	aPointList->Clear();
	aPointList->AddRange(resPointList);

	return IcPclResult::IC_PCL_SUCCEED;
}

IcPclResult IcPclProxyForRoboPath::FilterPointList(List<IcVector3D^>^ aPointList)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	List<IcVector3D^>^ resPointList = gcnew List<IcVector3D^>();

	cloud->points.resize(aPointList->Count);
	for (int i = 0; i < cloud->size(); i++) {
		pcl::PointXYZ& point = cloud->points[i];
		point.x = aPointList[i]->x;
		point.y = aPointList[i]->y;
		point.z = aPointList[i]->z;
	}

	// Fill in the cloud data
	//cloud->width = 5;
	//cloud->height = 1;
	//cloud->points.resize(cloud->width * cloud->height);

	for (auto& point : *cloud)
	{
		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cerr << "Cloud before filtering: " << std::endl;
	for (const auto& point : *cloud)
		std::cerr << "    " << point.x << " "
		<< point.y << " "
		<< point.z << std::endl;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 100.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	for (const auto& point : *cloud_filtered)
		std::cerr << "    " << point.x << " "
		<< point.y << " "
		<< point.z << std::endl;

	for (int i = 0; i < cloud_filtered->size(); i++) {

		IcVector3D^ icPoint = gcnew IcVector3D(
			cloud_filtered->points[i].x,
			cloud_filtered->points[i].y,
			cloud_filtered->points[i].z
		);

		resPointList->Add(icPoint);

	}

	// 結果を格納
	aPointList->Clear();
	aPointList->AddRange(resPointList);

	return IcPclResult::IC_PCL_SUCCEED;
}

IcPclResult IcPclProxyForRoboPath::ReadFilterPointList(String^ aFilePath, List<IcVector3D^>^ aPointList)
{
	List<IcVector3D^>^ resPointList = gcnew List<IcVector3D^>();
	IcPclResult res = ReadPointList(aFilePath, resPointList);
	res = FilterPointListStatisticalOutlinerRemoval(resPointList);
	aPointList->Clear();
	aPointList->AddRange(resPointList);
	return IcPclResult::IC_PCL_SUCCEED;
}

IcPclResult IcPclProxyForRoboPath::FilterPointListStatisticalOutlinerRemoval(List<IcVector3D^>^ aPointList)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	List<IcVector3D^>^ resPointList = gcnew List<IcVector3D^>();

	cloud->points.resize(aPointList->Count);
	for (int i = 0; i < cloud->size(); i++) {
		pcl::PointXYZ& point = cloud->points[i];
		point.x = aPointList[i]->x;
		point.y = aPointList[i]->y;
		point.z = aPointList[i]->z;
	}

	// Fill in the cloud data
	//cloud->width = 5;
	//cloud->height = 1;
	//cloud->points.resize(cloud->width * cloud->height);

	/*for (auto& point : *cloud)
	{
		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1024 * rand() / (RAND_MAX + 1.0f);
	}*/

	std::cerr << "Cloud before filtering: " << std::endl;
	for (const auto& point : *cloud)
		std::cerr << "    " << point.x << " "
		<< point.y << " "
		<< point.z << std::endl;

	// Create the filtering object
	/*pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 100.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filtered);*/

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	for (const auto& point : *cloud_filtered)
		std::cerr << "    " << point.x << " "
		<< point.y << " "
		<< point.z << std::endl;

	for (int i = 0; i < cloud_filtered->size(); i++) {

		IcVector3D^ icPoint = gcnew IcVector3D(
			cloud_filtered->points[i].x,
			cloud_filtered->points[i].y,
			cloud_filtered->points[i].z
		);

		resPointList->Add(icPoint);

	}

	// 結果を格納
	aPointList->Clear();
	aPointList->AddRange(resPointList);

	return IcPclResult::IC_PCL_SUCCEED;
}
