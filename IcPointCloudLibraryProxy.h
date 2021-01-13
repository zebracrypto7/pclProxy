// IcPointCloudLibraryProxy.h

#pragma once

#define generic THRUST_GENERIC		// �R���p�C���G���[�΍�
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/shot_omp.h>
#include <iostream>
#include <sstream>
#include <vector>
#undef generic						// �R���p�C���G���[�΍�
#include <pcl/filters/voxel_grid.h>

#include "IcPclConst.h"

using namespace System;
using namespace System::Collections::Generic;
using namespace System::Diagnostics;
using namespace IcCalcUtil;

namespace IcPointCloudLibraryProxy {

	public ref class IcPclProxyForRoboPath
	{
		// TODO: ���̃N���X�́A���[�U�[�̃��\�b�h�������ɒǉ����Ă��������B
	public:
		IcPclProxyForRoboPath();
		IcPclResult ReadPointList(String^ aFilePath, List<IcVector3D^>^ aPointList);
		IcPclResult ReadFilterPointList(String^ aFilePath, List<IcVector3D^>^ aPointList);
		IcPclResult FilterPointList(List<IcVector3D^>^ aPointList);
		IcPclResult FilterPointListStatisticalOutlinerRemoval(List<IcVector3D^>^ aPointList);

	};
}
