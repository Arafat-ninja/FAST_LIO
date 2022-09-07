#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
        :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{

    inf_bound = 10;      // 有效点集合,大于10m则是盲区
    N_SCANS = 6;         //多线激光雷达的线数
    SCAN_RATE = 10;
    group_size = 8;      // 8个点为一组
    disA = 0.01;         // 点集合的距离阈值,判断是否为平面
    disB = 0.1;          // 点集合的距离阈值,判断是否为平面
    p2l_ratio = 225;     // 点到线的距离阈值，需要大于这个值才能判断组成面
    limit_maxmid = 6.25; // 中点到左侧的距离变化率范围
    limit_midmin = 6.25; // 中点到右侧的距离变化率范围
    limit_maxmin = 3.24; // 左侧到右侧的距离变化率范围
    jump_up_limit = 170.0;
    jump_down_limit = 8.0;
    cos160 = 160.0;
    edgea = 2;   //点与点距离超过两倍则认为遮挡
    edgeb = 0.1; //点与点距离超过0.1m则认为遮挡
    smallp_intersect = 172.5;
    smallp_ratio = 1.2;        //三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面
    given_offset_time = false; //是否提供时间偏移

    jump_up_limit = cos(jump_up_limit / 180 * M_PI);       //角度大于170度的点跳过，认为在
    jump_down_limit = cos(jump_down_limit / 180 * M_PI);   //角度小于8度的点跳过
    cos160 = cos(cos160 / 180 * M_PI);                     //夹角限制
    smallp_intersect = cos(smallp_intersect / 180 * M_PI); //三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面

}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;
  
  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    //// 清除之前的点云缓存
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    double t1 = omp_get_wtime();
    int plsize = msg->point_num;
    // cout<<"plsie: "<<plsize<<endl;
    //// 预留空间
    pl_corn.reserve(plsize); // 角点
    pl_surf.reserve(plsize); // 面特征点
    pl_full.resize(plsize);  // 储存全部点(特征提取或间隔采样后）

    // 清空缓存内的点云并预留足够空间
    for(int i=0; i<N_SCANS; i++)
    {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
    }
    //有效点数
    uint valid_num = 0;

    //// 特征提取（默认不进行特征提取）
    if (feature_enabled)
    {

        // 分别对每个点云进行处理
        for(uint i=1; i<plsize; i++)
        {
            // 只取线数在0~N_SCANS内并且回波次序为0或者1的点云
            // https://zhuanlan.zhihu.com/p/461364648  关于tag信息
            if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

                bool is_new = false;
                //与前一点间距太小则忽略该点，间距太小不利于特征提取
                if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
                   || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
                   || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
                {
                    // 按照line划分点云
                    pl_buff[msg->points[i].line].push_back(pl_full[i]);  // 将当前点加入到对应line的pl_buff队列中
                }
            }
        }
        static int count = 0;
        static double time = 0.0;
        count ++;
        double t0 = omp_get_wtime();
        // 对每个line中的激光雷达分别进行处理
        for(int j=0; j<N_SCANS; j++)
        {
            // 如果该line中的点云过少，则继续处理下一条line
            if(pl_buff[j].size() <= 5) continue;
            pcl::PointCloud<PointType> &pl = pl_buff[j]; // 当前line的点云
            plsize = pl.size();
            vector<orgtype> &types = typess[j]; // 用于记录每个点的距离、角度、特征种类等属性
            types.clear();
            types.resize(plsize);
            plsize--;  //???
            for(uint i=0; i<plsize; i++)
            {
                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);  // 计算每个点到机器人本身的xy平面投影距离
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);  // 计算两个间隔点的距离
            }
            //因为i最后一个点没有i+1了所以就单独求了一个range，没有distance
            types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
            give_feature(pl, types); //通过点和每个点的属性，计算特征
            // pl_surf += pl;
        }
        time += omp_get_wtime() - t0;
        printf("Feature extraction time: %lf \n", time / count);
    }
    else
    {
        // 分别对每个点云进行处理
        for(uint i=1; i<plsize; i++)
        {
            // 只取线数在0~N_SCANS内并且回波次序为0或者1的点云
            if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {

                valid_num ++;  // 有效的点云数
                // 等间隔降采样
                if (valid_num % point_filter_num == 0)
                {
                    pl_full[i].x = msg->points[i].x;
                    pl_full[i].y = msg->points[i].y;
                    pl_full[i].z = msg->points[i].z;
                    pl_full[i].intensity = msg->points[i].reflectivity;
                    pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

                    // 只有当当前点和上一点的间距足够大（>1e-7），并且在最小距离阈值之外，才将当前点认为是有用的点，加入到pl_surf队列中
                    if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
                       || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
                       || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)
                          && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
                    {
                        pl_surf.push_back(pl_full[i]);
                    }
                }
            }
        }
    }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;
      if(pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)//todo check pl_orig.points[plsize - 1].time
    {
        given_offset_time = true;
    }
    else
    {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578; // 记录第一个点(index 0)的yaw， to degree
        double yaw_end  = yaw_first;
        int layer_first = pl_orig.points[0].ring; // 第一个点(index 0)的layer序号
        for (uint i = plsize - 1; i > 0; i--) // 倒序遍历，找到与第一个点相同layer的最后一个点
        {
            if (pl_orig.points[i].ring == layer_first)
            {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;// 与第一个点相同layer的最后一个点的yaw
                break;
            }
        }
    }

    if(feature_enabled)
    {
        for (int i = 0; i < N_SCANS; i++)
        {
            pl_buff[i].clear();
            pl_buff[i].reserve(plsize);
        }

        //计算时间、转换点云格式为PointType，正序遍历
        for (int i = 0; i < plsize; i++)
        {
            PointType added_pt;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            int layer  = pl_orig.points[i].ring;
            if (layer >= N_SCANS) continue;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms

            if (!given_offset_time)
            {
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957; // 但前点yaw, to degree
                if (is_first[layer]) // 如果当前点是其对应layer的第一个点
                {
                    // printf("layer: %d; is first: %d", layer, is_first[layer]);
                    yaw_fp[layer]=yaw_angle; // 记录为当前点对应layer的起始yaw
                    is_first[layer]=false;
                    added_pt.curvature = 0.0; //当前点curvature（时间）置零
                    yaw_last[layer]=yaw_angle; // 暂时记录为当前点对应layer的结束yaw
                    time_last[layer]=added_pt.curvature;
                    continue;
                }

                if (yaw_angle <= yaw_fp[layer])
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
                }
                else
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
                }

                if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

                yaw_last[layer] = yaw_angle; // 记录当前layer最后一个点的yaw
                time_last[layer]=added_pt.curvature; //  记录当前layer最后一个点的时间
            }

            pl_buff[layer].points.push_back(added_pt);
        }

        for (int j = 0; j < N_SCANS; j++)
        {
            PointCloudXYZI &pl = pl_buff[j]; // points_line
            int linesize = pl.size();
            if (linesize < 2) continue;
            vector<orgtype> &types = typess[j]; //用于记录当前扫描线上每个点的参数
            types.clear();
            types.resize(linesize);
            linesize--;
            for (uint i = 0; i < linesize; i++)
            {
                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
            give_feature(pl, types);
        }
    }
    else
    {
        for (int i = 0; i < plsize; i++)
        {
            PointType added_pt;
            // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature = pl_orig.points[i].time / 1000.0;  // curvature unit: ms

            if (!given_offset_time)
            {
                int layer = pl_orig.points[i].ring;
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

                if (is_first[layer])
                {
                    // printf("layer: %d; is first: %d", layer, is_first[layer]);
                    yaw_fp[layer]=yaw_angle;
                    is_first[layer]=false;
                    added_pt.curvature = 0.0;
                    yaw_last[layer]=yaw_angle;
                    time_last[layer]=added_pt.curvature;
                    continue;
                }

                // compute offset time
                if (yaw_angle <= yaw_fp[layer])
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
                }
                else
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
                }

                if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

                yaw_last[layer] = yaw_angle;
                time_last[layer]=added_pt.curvature;
            }

            if (i % point_filter_num == 0)
            {
                if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
                {
                    pl_surf.points.push_back(added_pt);
                }
            }
        }
    }
}

/**
 * @brief 特征提取
 * @param
 *      pl：     一条 line 的点云信息
 *      types：  这条 line 对应点云的属性
 * @return void
 */
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
    int plsize = pl.size(); // 输入扫描线的原始点数
    int plsize2; // 用于估计特征的点数
    if(plsize == 0)
    {
        printf("something wrong\n");
        return;
    }
    uint head = 0;
    //// 更新head为第一个大于blind范围的点索引
    while(types[head].range < blind)
    {
        head++;
    }

    // group_size默认等于8
    plsize2 = (plsize > group_size) ? (plsize - group_size) : 0; // 判断当前点后面是否还有8个点

    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

    uint i_nex = 0, i2;
    uint last_i = 0; uint last_i_nex = 0;
    int last_state = 0;
    int plane_type;

    //// 第一个大于blind范围的点索引开始遍历,判断平面特征
    for(uint i=head; i<plsize2; i++)
    {
        if(types[i].range < blind) // blind范围内直接跳过
        {
            continue;
        }

        i2 = i; //i2记录当前点索引

        // 输出
        // i_nex 局部最后最后一个点的索引
        // curr_direct 归一化后的，局部范围最后一个点与第一个点的坐标差值，即向量（i_cur --> i_nex)
        // return 1 正常退出， 0 中途break，curr_direct置零
        plane_type = plane_judge(pl, types, i, i_nex, curr_direct);  // 求平面，并返回类型0 1 2

        //返回1一般默认是平面
        if(plane_type == 1) //plane_judge正常退出
        {
            // 设置确定的平面点和可能的平面点
            for(uint j=i; j<=i_nex; j++)
            {
                if(j!=i && j!=i_nex)
                {
                    types[j].ftype = Real_Plane; // 局部范围内部点 定义为real平面点
                }
                else
                {
                    types[j].ftype = Poss_Plane; // 局部范围边界点 定义可能平面
                }
            }

            // if(last_state==1 && fabs(last_direct.sum())>0.5)
            if(last_state==1 && last_direct.norm()>0.1) // 根据上一状态（局部是平面）和长度（向量模长），决定起始点的类型
            {
                double mod = last_direct.transpose() * curr_direct;
                if(mod>-0.707 && mod<0.707)
                {
                    // 两个面法向量夹角在45度和135度之间 认为是两平面边缘上的点
                    types[i].ftype = Edge_Plane; //平面交接的边
                }
                else
                {
                    //否则认为是真正的平面点
                    types[i].ftype = Real_Plane;//平面
                }
            }

            i = i_nex - 1;
            last_state = 1;
        }
        else // if(plane_type == 2)
        {
            // plane_type=0或2的时候
            i = i_nex;
            last_state = 0;
        }
        // else if(plane_type == 0)
        // {
        //   if(last_state == 1)
        //   {
        //     uint i_nex_tem;
        //     uint j;
        //     for(j=last_i+1; j<=last_i_nex; j++)
        //     {
        //       uint i_nex_tem2 = i_nex_tem;
        //       Eigen::Vector3d curr_direct2;

        //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

        //       if(ttem != 1)
        //       {
        //         i_nex_tem = i_nex_tem2;
        //         break;
        //       }
        //       curr_direct = curr_direct2;
        //     }

        //     if(j == last_i+1)
        //     {
        //       last_state = 0;
        //     }
        //     else
        //     {
        //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
        //       {
        //         if(k != i_nex_tem)
        //         {
        //           types[k].ftype = Real_Plane;
        //         }
        //         else
        //         {
        //           types[k].ftype = Poss_Plane;
        //         }
        //       }
        //       i = i_nex_tem-1;
        //       i_nex = i_nex_tem;
        //       i2 = j-1;
        //       last_state = 1;
        //     }

        //   }
        // }
        // 更新为last状态
        last_i = i2;
        last_i_nex = i_nex;
        last_direct = curr_direct;
    }

    plsize2 = plsize > 3 ? plsize - 3 : 0;
    //// 从head+3向后遍历，判断非平面点是不是edge，以及判断edge的类型
    for(uint i=head+3; i<plsize2; i++)
    {
        if(types[i].range<blind || types[i].ftype>=Real_Plane) // 距离太近或这已经判断为edge或者平面
        {
            continue;
        }

        if(types[i-1].dista<1e-16 || types[i].dista<1e-16) //前两个点间距太小
        {
            continue;
        }

        Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z); // 当前点组成的向量
        Eigen::Vector3d vecs[2]; // 从当前点指向前后相邻点的两个向量

        // 更新当前点 前后两点的属性并保存进 types[i].edj
        for(int j=0; j<2; j++) // 计算当前点与前一点、后一点的向量，判断当前点前后两个方向的edge属性
        {
            int m = -1;
            if(j == 1)
            {
                m = 1;
            }
            //若当前的 前（m = -1)/ 后（m = 1)  有点在盲区内
            if(types[i+m].range < blind)
            {
                if(types[i].range > inf_bound) // 根据当前点平面距离，判断edge jump属性
                {
                    types[i].edj[j] = Nr_inf; // 赋予该点Nr_inf(跳变较远)
                }
                else
                {
                    types[i].edj[j] = Nr_blind; // 赋予该点Nr_blind(在盲区)
                }
                continue;
            }

            vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
            vecs[j] = vecs[j] - vec_a; // 当前点 指向 前/后点 的向量

            types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm(); // cos(当前点指向前一点或后一点的向量的夹角)
            if(types[i].angle[j] < jump_up_limit) // jump_up_limit 默认cos170度，即夹角接近180度
            {
                types[i].edj[j] = Nr_180; // 更新前后点的属性
            }
            else if(types[i].angle[j] > jump_down_limit) //jump_down_limit 默认8度，即夹角接近0度
            {
                types[i].edj[j] = Nr_zero;
            }
        }

        // 更新当前点与相邻两点的夹角cos值
        types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();

        // 根据 前端点edge jump类型，后端点edge jump类型，与后一点间距，与前一点间距的4倍，判断edge的类型
        if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
        {
            //前一个点正常 && 后一个点在激光束上 && 当前点后一个点的距离大于0.0225m && 当前点与后一个点距离 大于 前一个点与当前点的距离的四倍
            if(types[i].intersect > cos160)  // 夹角小于160度
            {
                if(edge_jump_judge(pl, types, i, Prev))
                {
                    types[i].ftype = Edge_Jump;
                }
            }
        }
        else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
        {
            //前一个点在激光束上 && 后一个点正常 && 前一个点与当前点的距离大于0.0225m && 前一个点与当前点的距离大于当前点与后一个点距离的四倍
            if(types[i].intersect > cos160)  // 夹角小于160度
            {
                if(edge_jump_judge(pl, types, i, Next))
                {
                    types[i].ftype = Edge_Jump;
                }
            }
        }
        else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
        {
            //前面的是正常点 && (当前点到中心距离>10m并且后点在盲区)
            if(edge_jump_judge(pl, types, i, Prev))
            {
                types[i].ftype = Edge_Jump;
            }
        }
        else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
        {
            //(当前点到中心距离>10m并且前点在盲区) && 后面的是正常点
            if(edge_jump_judge(pl, types, i, Next))
            {
                types[i].ftype = Edge_Jump;
            }

        }
        else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
        {
            //前后点都不是正常点
            if(types[i].ftype == Nor)
            {
                types[i].ftype = Wire;
            }
        }
    }

    plsize2 = plsize-1;
    double ratio; // 点的前后间距比，大：小

    //// 还剩下来一些正常点继续找平面点
    for(uint i=head+1; i<plsize2; i++)
    {
        //前面、当前、之后三个点都需要不在盲区内
        if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
        {
            continue;
        }
        // 前面和当前 当前和之后的点与点之间距离都不能太近
        if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
        {
            continue;
        }


        if(types[i].ftype == Nor)
        {
            // 求点与点间距的比例 大间距/小间距
            if(types[i-1].dista > types[i].dista)
            {
                ratio = types[i-1].dista / types[i].dista;
            }
            else
            {
                ratio = types[i].dista / types[i-1].dista;
            }

            //smallp_intersect：默认172.5度余弦值
            //smallp_intersect：默认1.2
            //如果夹角大于172.5度 && 间距比例<1.2, 前后三个点认为是真平面点
            if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
            {
                if(types[i-1].ftype == Nor)
                {
                    types[i-1].ftype = Real_Plane;
                }
                if(types[i+1].ftype == Nor)
                {
                    types[i+1].ftype = Real_Plane;
                }
                types[i].ftype = Real_Plane;
            }
        }
    }

    int last_surface = -1; //用于记录上一面特征的索引
    //// 从头遍历 存储平面点 角点，填充 pl_surf, pl_corn 特征容器
    for(uint j=head; j<plsize; j++)
    {
        // 可能的平面点和确定的平面点
        if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
        {
            if(last_surface == -1)
            {
                last_surface = j;
            }
            //  间隔采样 滤波 从每次新找到面点开始每几个点才取一个
            if(j == uint(last_surface+point_filter_num-1))
            {
                PointType ap;
                ap.x = pl[j].x;
                ap.y = pl[j].y;
                ap.z = pl[j].z;
                ap.intensity = pl[j].intensity;
                ap.curvature = pl[j].curvature;
                pl_surf.push_back(ap);//记录

                last_surface = -1;
            }
        }
        else
        {
            // 跳变较大的边缘边的点 位于平面边缘的点
            if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
            {
                pl_corn.push_back(pl[j]); // 记录edge特征
            }
            // 假如上次找到的面点被滤波掉了，而此时已经到了边缘
            if(last_surface != -1)
            {
                //edge特征和上一个surface特征之间取所有点均值，作为面特征
                PointType ap;
                for(uint k=last_surface; k<j; k++)
                {
                    ap.x += pl[k].x;
                    ap.y += pl[k].y;
                    ap.z += pl[k].z;
                    ap.intensity += pl[k].intensity;
                    ap.curvature += pl[k].curvature;
                }
                ap.x /= (j-last_surface);
                ap.y /= (j-last_surface);
                ap.z /= (j-last_surface);
                ap.intensity /= (j-last_surface);
                ap.curvature /= (j-last_surface);
                pl_surf.push_back(ap);
            }
            last_surface = -1;
        }
    }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

/**
 * @brief  平面判断
 * @param
 *      pl：         一条 line 的点云信息
 *      types：      这条 line 对应点云的属性
 *      i_cur：      当前点索引
 *      i_nex：      局部范围内最后一个点的索引
 *      curr_direct： 方向向量
 * @return int
 */
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
    // 用于限制特征点计算时，局部范围的阈值
    double group_dis = disA*types[i_cur].range + disB; // 0.01*sqrt(x^2+y^2)+0.1 基本上可以近似看成是0.1 100m的时候才到0.2
    group_dis = group_dis * group_dis;
    // i_nex = i_cur;

    double two_dis;
    vector<double> disarr; // 前后点距离数组（line内相邻有效点间距）
    disarr.reserve(20);

    //group_size(默认8）， 索引范围内遍历，先取够8个点
    for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
    {
        if(types[i_nex].range < blind) // group_size存在点的平面距离小于blind，将法向量设置为零向量，直接返回2
        {
            curr_direct.setZero();
            return 2;
        }
        disarr.push_back(types[i_nex].dista); // 保存与后一个点的距离
    }

    for(;;) // 继续向后遍历，保存group_dis范围以内，与后一个点的间距,记录最后一个局部点索引为i_nex
    {
        if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;  // 索引超出所有点的个数直接 break

        if(types[i_nex].range < blind)
        {
            //距离雷达原点太小了将法向量设置为零向量
            curr_direct.setZero();
            return 2;
        }
        vx = pl[i_nex].x - pl[i_cur].x;
        vy = pl[i_nex].y - pl[i_cur].y;
        vz = pl[i_nex].z - pl[i_cur].z;
        two_dis = vx*vx + vy*vy + vz*vz; // 最后一个点与当前点的距离平方和
        if(two_dis >= group_dis) // 超出局部间距距离范围约束时，跳出
        {
            break;
        }
        disarr.push_back(types[i_nex].dista);
        i_nex++;
    }

    double leng_wid = 0; // 记录局部范围内，叉乘的最大模长，即局部范围内最大平行四边形面积
    double v1[3], v2[3];
    for(uint j=i_cur+1; j<i_nex; j++) //局部范围内，从当前点向后遍历，i_nex为局部范围内索引最大值
    {

        if((j >= pl.size()) || (i_cur >= pl.size())) break;
        //计算向量（i_cur --> j）
        v1[0] = pl[j].x - pl[i_cur].x;
        v1[1] = pl[j].y - pl[i_cur].y;
        v1[2] = pl[j].z - pl[i_cur].z;
        // vx,vy,vz为局部范围最后一个点与第一个点的坐标差值
        // 向量（i_cur --> i_nex) 和 向量（i_cur --> j） 叉乘
        v2[0] = v1[1]*vz - vy*v1[2];
        v2[1] = v1[2]*vx - v1[0]*vz;
        v2[2] = v1[0]*vy - vx*v1[1];

        double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2]; // 叉乘的模长，平行四边形面积平方

        //寻找最大面积的平方(也就是寻找距离（i_cur --> i_nex)最远的点j)
        if(lw > leng_wid)
        {
            leng_wid = lw;
        }
    }

    // | i_cur --> i_nex| = |AC|
    //|AC|*|AC|/(|AC|*|AC|*h*h)<225
    //也就是h>1/15 最远点到AC的距离要大于0.06667m
    //太近了不好拟合一个平面
    if((two_dis*two_dis/leng_wid) < p2l_ratio)
    {
        curr_direct.setZero();
        return 0;
    }

    uint disarrsize = disarr.size();
    for(uint j=0; j<disarrsize-1; j++) // 排序，disarr按从大到小，leng_wid为最小的相邻点间隔距离
    {
        for(uint k=j+1; k<disarrsize; k++)
        {
            if(disarr[j] < disarr[k])
            {
                leng_wid = disarr[j];
                disarr[j] = disarr[k];
                disarr[k] = leng_wid; //j、k互换
            }
        }
    }

    if(disarr[disarr.size()-2] < 1e-16) // 第二小的点间距
    {
        curr_direct.setZero();
        return 0;
    }

    if(lidar_type==AVIA)
    {
        // 点与点之间距离变化太大的时候 可能与激光束是平行的 舍弃
        double dismax_mid = disarr[0]/disarr[disarrsize/2];
        double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];
        if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
        {
            curr_direct.setZero();
            return 0;
        }
    }
    else
    {
        double dismax_min = disarr[0] / disarr[disarrsize-2]; // 最大最小间距比
        if(dismax_min >= limit_maxmin)
        {
            curr_direct.setZero();
            return 0;
        }
    }

    curr_direct << vx, vy, vz; // vx,vy,vz为局部范围最后一个点与第一个点的坐标差值，即向量（i_cur --> i_nex)
    curr_direct.normalize();
    return 1;
}

/**
 * @brief  边缘判断
 * @param
 *      pl：         一条 line 的点云信息
 *      types：      这条 line 对应点云的属性
 *      i：          当前点索引
 *      nor_dir：    方向，前或后
 * @return 是否是 edge
 */
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
    if(nor_dir == 0)
    {
        // 前两个点不能在盲区
        if(types[i-1].range<blind || types[i-2].range<blind)
        {
            return false;
        }
    }
    else if(nor_dir == 1)
    {
        // 后两个点不能在盲区
        if(types[i+1].range<blind || types[i+2].range<blind)
        {
            return false;
        }
    }
    // d1,d2为向前或向后的相邻点间距，通过nor_dir控制方向
    double d1 = types[i+nor_dir-1].dista;
    double d2 = types[i+3*nor_dir-2].dista;
    double d;

    // 将大小间距进行调换 d1大 d2小
    if(d1<d2)
    {
        d = d1;
        d1 = d2;
        d2 = d;
    }

    d1 = sqrt(d1);
    d2 = sqrt(d2);

    //间距大于edgea(默认2)倍 或 间距差大于edgeb(默认0.1)
    if(d1>edgea*d2 || (d1-d2)>edgeb)
    {
        //假如间距太大 可能是被遮挡，就不把它当作边缘点
        return false;
    }

    return true;
}
