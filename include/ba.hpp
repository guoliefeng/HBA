/**
 * @file ba.hpp
 * @brief 激光雷达 Bundle Adjustment 核心：体素八叉树、Hessian 计算、LM 阻尼优化
 *
 * 优化目标：最小化各平面体素点云协方差矩阵的最小特征值之和 (BALM 系列方法)
 * 主要类：
 *   - VOX_HESS    : 收集平面体素并计算残差/雅可比/Hessian
 *   - OCTO_TREE_* : 体素八叉树，自适应细分至平面叶子
 *   - VOX_OPTIMIZER: Levenberg-Marquardt 迭代求解器
 */
#ifndef BA_HPP
#define BA_HPP

#include <thread>
#include <fstream>
#include <iomanip>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "tools.hpp"

#define WIN_SIZE 10       // 滑动窗口大小（帧数）
#define GAP 5             // 相邻窗口起点间隔
#define AVG_THR             // 使用平均残差作为收敛判据
#define FULL_HESS             // 存储完整 Hessian 对角近似供 PGO 使用
// #define ENABLE_RVIZ         // 开启 RViz 可视化（平面法向量等）
// #define ENABLE_FILTER       // 开启体素滤波

const double one_three = (1.0 / 3.0);

int layer_limit = 2;    // 八叉树最大细分层数
int MIN_PT = 15;        // 体素最少点数，低于此值丢弃
int thd_num = 16;       // Hessian 累加时的并行线程数

/**
 * @brief 体素 Hessian 容器
 *
 * 收集所有判定为平面的体素，提供：
 *   - 残差计算（最小特征值之和）
 *   - 解析 Jacobian / Hessian 累加 (acc_evaluate2)
 *   - 离群体素剔除 (remove_residual)
 */
class VOX_HESS
{
public:
  vector<const vector<VOX_FACTOR>*> plvec_voxels;    // 各平面体素在各帧的 VOX_FACTOR 统计 (雷达系)
  vector<PLV(3)> origin_points;                      // 各帧原始点（ENABLE_FILTER 时使用）
  int win_size;                                      // 滑窗大小

  VOX_HESS(int _win_size = WIN_SIZE): win_size(_win_size){origin_points.resize(win_size);}

  ~VOX_HESS()
  {
    vector<const vector<VOX_FACTOR>*>().swap(plvec_voxels);
  }

  /***************************************************
  * @brief 将vec_orig保存到origin_points_
  * @detail  
  * @param[in] vec_orig: 点云数据
  * @param[out] origin_points_: 点云数据
  * @return 无
  ****************************************************/
  void get_center(const PLV(3)& vec_orig, PLV(3)& origin_points_)
  {
    size_t pt_size = vec_orig.size();
    for(size_t i = 0; i < pt_size; i++)
      origin_points_.emplace_back(vec_orig[i]);
    return;
  }

  /***************************************************
  * @brief 将sig_orig和vec_orig保存到内部变量 origin_points 和 plvec_voxels
  * @detail  
  * @param[in] sig_orig: 体素数据
  * @param[in] vec_orig: 点云数据
  * @return 无
  ****************************************************/
  void push_voxel(const vector<VOX_FACTOR>* sig_orig, const vector<PLV(3)>* vec_orig)
  {
    int process_size = 0;
    for(int i = 0; i < win_size; i++)
      if((*sig_orig)[i].N != 0)
        process_size++;

    #ifdef ENABLE_FILTER
    if(process_size < 1) return;

    for(int i = 0; i < win_size; i++)
      if((*sig_orig)[i].N != 0)
        get_center((*vec_orig)[i], origin_points[i]);
    #endif
    
    if(process_size < 2) return;
    
    plvec_voxels.push_back(sig_orig);
  }

  /** @brief 构造对称矩阵外积 u_m * u_n^T 的 6 维向量表示（用于 Hessian 推导） */
  Eigen::Matrix<double, 6, 1> lam_f(Eigen::Vector3d *u, int m, int n)
  {
    Eigen::Matrix<double, 6, 1> jac;
    jac[0] = u[m][0] * u[n][0];
    jac[1] = u[m][0] * u[n][1] + u[m][1] * u[n][0];
    jac[2] = u[m][0] * u[n][2] + u[m][2] * u[n][0];
    jac[3] = u[m][1] * u[n][1];
    jac[4] = u[m][1] * u[n][2] + u[m][2] * u[n][1];
    jac[5] = u[m][2] * u[n][2];
    return jac;
  }

  /**
   * @brief 对 [head, end) 范围内的体素累加 Hessian、Jacobian 和残差
   *
   * 对每个体素：
   *   1. 将各帧 sig_orig 变换到世界系并合并
   *   2. 计算合并点云协方差的最小特征值 λ₀ 作为残差
   *   3. 对 λ₀ 关于各帧位姿 (R, p) 求导，累加 6×6 Hessian 块
   *
   * @param xs  滑窗内各帧位姿
   * @param head, end  体素索引范围（用于多线程划分）
   * @param Hess  输出 Hessian 矩阵 (6*win_size × 6*win_size)
   * @param JacT  输出 Jacobian 转置乘以残差
   * @param residual  输出残差（最小特征值之和）
   */
  void acc_evaluate2(const vector<IMUST>& xs, int head, int end,
                     Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, double& residual)
  {
    Hess.setZero(); JacT.setZero(); residual = 0;
    vector<VOX_FACTOR> sig_tran(win_size);    //map坐标系
    const int kk = 0;

    PLV(3) viRiTuk(win_size);
    PLM(3) viRiTukukT(win_size);

    vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>> Auk(win_size);  //vector<3*6>
    Eigen::Matrix3d umumT;

    for(int a = head; a < end; a++)
    {
      const vector<VOX_FACTOR>& sig_orig = *plvec_voxels[a];

      VOX_FACTOR sig;
      for(int i = 0; i < win_size; i++)
      {
        if(sig_orig[i].N != 0)
        {
          sig_tran[i].transform(sig_orig[i], xs[i]);
          sig += sig_tran[i];
        }
      }
      
      const Eigen::Vector3d& vBar = sig.v / sig.N;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P/sig.N - vBar * vBar.transpose());
      const Eigen::Vector3d& lmbd = saes.eigenvalues();
      const Eigen::Matrix3d& U = saes.eigenvectors();
      int NN = sig.N;
      
      Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};    //特征向量*3

      const Eigen::Vector3d& uk = u[kk];              // 最小特征值对应的特征向量
      Eigen::Matrix3d ukukT = uk * uk.transpose();
      // 最小特征值对协方差矩阵的导数（除最小特征值外各特征方向的贡献）
      umumT.setZero();
      for(int i = 0; i < 3; i++)
      {
        if(i != kk)
          umumT += 2.0/(lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();
      }

      for(int i = 0; i < win_size; i++)
      {
        if(sig_orig[i].N != 0)
        {
          Eigen::Matrix3d Pi = sig_orig[i].P;  //协方差N*A
          Eigen::Vector3d vi = sig_orig[i].v;  //均值N*P
          Eigen::Matrix3d Ri = xs[i].R;        //旋转矩阵
          double ni = sig_orig[i].N;

          Eigen::Matrix3d vihat; vihat << SKEW_SYM_MATRX(vi);
          Eigen::Vector3d RiTuk = Ri.transpose() * uk;
          Eigen::Matrix3d RiTukhat; RiTukhat << SKEW_SYM_MATRX(RiTuk);

          Eigen::Vector3d PiRiTuk = Pi * RiTuk;
          viRiTuk[i] = vihat * RiTuk;
          viRiTukukT[i] = viRiTuk[i] * uk.transpose();
          
          Eigen::Vector3d ti_v = xs[i].p - vBar;
          double ukTti_v = uk.dot(ti_v);

          Eigen::Matrix3d combo1 = hat(PiRiTuk) + vihat * ukTti_v;
          Eigen::Vector3d combo2 = Ri*vi + ni*ti_v;
          Auk[i].block<3, 3>(0, 0) = (Ri*Pi + ti_v*vi.transpose()) * RiTukhat - Ri*combo1;
          Auk[i].block<3, 3>(0, 3) = combo2 * uk.transpose() + combo2.dot(uk) * I33;
          Auk[i] /= NN;

          const Eigen::Matrix<double, 6, 1> &jjt = Auk[i].transpose() * uk;
          JacT.block<6, 1>(6*i, 0) += jjt;

          const Eigen::Matrix3d &HRt = 2.0/NN * (1.0-ni/NN) * viRiTukukT[i];
          Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[i];
          Hb.block<3, 3>(0, 0) +=
            2.0/NN*(combo1-RiTukhat*Pi)*RiTukhat - 2.0/NN/NN*viRiTuk[i]*viRiTuk[i].transpose() - 0.5*hat(jjt.block<3, 1>(0, 0));
          Hb.block<3, 3>(0, 3) += HRt;
          Hb.block<3, 3>(3, 0) += HRt.transpose();
          Hb.block<3, 3>(3, 3) += 2.0/NN * (ni - ni*ni/NN) * ukukT;

          Hess.block<6, 6>(6*i, 6*i) += Hb;
        }
      }
      
      for(int i = 0; i < win_size-1; i++)
      {
        if(sig_orig[i].N != 0)
        {
          double ni = sig_orig[i].N;
          for(int j = i+1; j < win_size; j++)
            if(sig_orig[j].N != 0)
            {
              double nj = sig_orig[j].N;
              Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[j];
              Hb.block<3, 3>(0, 0) += -2.0/NN/NN * viRiTuk[i] * viRiTuk[j].transpose();
              Hb.block<3, 3>(0, 3) += -2.0*nj/NN/NN * viRiTukukT[i];
              Hb.block<3, 3>(3, 0) += -2.0*ni/NN/NN * viRiTukukT[j].transpose();
              Hb.block<3, 3>(3, 3) += -2.0*ni*nj/NN/NN * ukukT;

              Hess.block<6, 6>(6*i, 6*j) += Hb;
            }
        }
      }
      
      residual += lmbd[kk];
    }

    for(int i = 1; i < win_size; i++)
      for(int j = 0; j < i; j++)
        Hess.block<6, 6>(6*i, 6*j) = Hess.block<6, 6>(6*j, 6*i).transpose();
  }

  /***************************************************
  * @brief 计算残差
  * @detail  
  * @param[in] xs: 位姿
  * @param[in] residual: 残差，即各体素的点云协方差最小特征值之和
  * @return 无
  ****************************************************/
  void evaluate_only_residual(const vector<IMUST>& xs, double& residual)
  {
    residual = 0;
    vector<VOX_FACTOR> sig_tran(win_size);
    int kk = 0; // The kk-th lambda value

    int gps_size = plvec_voxels.size();

    for(int a = 0; a < gps_size; a++)
    {
      const vector<VOX_FACTOR>& sig_orig = *plvec_voxels[a];
      VOX_FACTOR sig;

      for(int i = 0; i < win_size; i++)
      {
        sig_tran[i].transform(sig_orig[i], xs[i]);
        sig += sig_tran[i];
      }

      Eigen::Vector3d vBar = sig.v / sig.N;
      Eigen::Matrix3d cmt = sig.P/sig.N - vBar * vBar.transpose();

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cmt);
      Eigen::Vector3d lmbd = saes.eigenvalues();

      residual += lmbd[kk];
    }
  }

  /***************************************************
  * @brief 计算每个体素的最小特征值
  * @detail  
  * @param[in] xs: 各帧位姿
  * @return 
  ****************************************************/
  std::vector<double> evaluate_residual(const vector<IMUST>& xs)
  {
    /* for outlier removal usage */
    std::vector<double> residuals;
    vector<VOX_FACTOR> sig_tran(win_size);
    int kk = 0; // The kk-th lambda value
    int gps_size = plvec_voxels.size();

    for(int a = 0; a < gps_size; a++)
    {
      const vector<VOX_FACTOR>& sig_orig = *plvec_voxels[a];
      VOX_FACTOR sig;

      for(int i = 0; i < win_size; i++)
      {
        sig_tran[i].transform(sig_orig[i], xs[i]);
        sig += sig_tran[i];
      }

      Eigen::Vector3d vBar = sig.v / sig.N;
      Eigen::Matrix3d cmt = sig.P / sig.N - vBar * vBar.transpose();

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cmt);
      Eigen::Vector3d lmbd = saes.eigenvalues();

      residuals.push_back(lmbd[kk]);
    }

    return residuals;
  }

  /***************************************************
  * @brief 根据特征值移除对应的体素数据
  * @detail  
  * @param[in] xs: 位姿
  * @param[in] threshold: 阈值
  * @param[in] reject_num: 最大移除数量
  * @return 无
  ****************************************************/
  void remove_residual(const vector<IMUST>& xs, double threshold, double reject_num)
  {
    vector<VOX_FACTOR> sig_tran(win_size);
    int kk = 0; // The kk-th lambda value
    int rej_cnt = 0;
    size_t i = 0;
    for(; i < plvec_voxels.size();)
    {
      const vector<VOX_FACTOR>& sig_orig = *plvec_voxels[i];
      VOX_FACTOR sig;

      for(int j = 0; j < win_size; j++)
      {
        sig_tran[j].transform(sig_orig[j], xs[j]);
        sig += sig_tran[j];
      }

      Eigen::Vector3d vBar = sig.v / sig.N;
      Eigen::Matrix3d cmt = sig.P / sig.N - vBar * vBar.transpose();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cmt);
      Eigen::Vector3d lmbd = saes.eigenvalues();

      if(lmbd[kk] >= threshold)
      {
        plvec_voxels.erase(plvec_voxels.begin()+i);
        rej_cnt++;
        continue;
      }
      i++;
      if(rej_cnt == reject_num) break;
    }
  }
};

int BINGO_CNT = 0;

/** @brief 八叉树节点状态 */
enum OCTO_STATE {UNKNOWN, MID_NODE, PLANE};

/**
 * @brief 体素八叉树节点
 *
 * 自适应细分流程 (recut)：
 *   UNKNOWN -> 点数不足 -> MID_NODE (丢弃)
 *           -> 判定为平面 -> PLANE (参与 BA)
 *           -> 非平面且未达 layer_limit -> 八叉细分 -> 递归 recut
 *           -> 非平面且已达 layer_limit -> MID_NODE (丢弃)
 */
class OCTO_TREE_NODE
{
public:
  OCTO_STATE octo_state;
  int layer, win_size;                               // 八叉树层号 / 滑窗大小
  vector<PLV(3)> vec_orig, vec_tran;                 // 各帧点云 (雷达系 / 世界系)
  vector<VOX_FACTOR> sig_orig, sig_tran;             // 各帧增量统计 (雷达系 / 世界系)

  OCTO_TREE_NODE* leaves[8];                         // 8 个子节点
  float voxel_center[3];                           // 体素中心坐标
  float quater_length;                             // 体素边长 / 4
  float eigen_thr;                                   // 平面判定特征值比阈值

  Eigen::Vector3d center, direct, value_vector;      // 质心 / 最小特征向量 / 特征值
  double eigen_ratio;                                // λ_min / λ_max
  
  #ifdef ENABLE_RVIZ
  ros::NodeHandle nh;
  ros::Publisher pub_residual = nh.advertise<sensor_msgs::PointCloud2>("/residual", 1000);
  ros::Publisher pub_direct = nh.advertise<visualization_msgs::MarkerArray>("/direct", 1000);
  #endif

  OCTO_TREE_NODE(int _win_size = WIN_SIZE, float _eigen_thr = 1.0/10):
    win_size(_win_size), eigen_thr(_eigen_thr)
  {
    octo_state = UNKNOWN; layer = 0;
    vec_orig.resize(win_size); vec_tran.resize(win_size);
    sig_orig.resize(win_size); sig_tran.resize(win_size);
    for(int i = 0; i < 8; i++)
      leaves[i] = nullptr;
  }

  virtual ~OCTO_TREE_NODE()
  {
    for(int i = 0; i < 8; i++)
      if(leaves[i] != nullptr)
        delete leaves[i];
  }

  /***************************************************
  * @brief 判断滑窗内所有帧的点云集合是否为平面
  * @detail  最小特征值/最大特征值, 判断阈值为: eigen_thr, 且没有不平衡
  * @param 无
  * @return true: 平面; false: 非平面
  ****************************************************/
  bool judge_eigen()
  {
    VOX_FACTOR covMat;
    for(int i = 0; i < win_size; i++)
      if(sig_tran[i].N > 0)
        covMat += sig_tran[i];
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat.cov());
    value_vector = saes.eigenvalues();    //特征值
    center = covMat.v / covMat.N;         //质心坐标
    direct = saes.eigenvectors().col(0);  //最小特征值对应的特征向量

    eigen_ratio = saes.eigenvalues()[0] / saes.eigenvalues()[2]; // [0] is the smallest
    if(eigen_ratio > eigen_thr) return 0;

    double eva0 = saes.eigenvalues()[0];
    double sqr_eva0 = sqrt(eva0);
    // 沿法向扰动中心，用于检测平面是否"平衡"（避免误判非平面结构）
    Eigen::Vector3d center_turb = center + 5 * sqr_eva0 * direct;
    vector<VOX_FACTOR> covMats(8);
    for(int i = 0; i < win_size; i++)
    {
      for(Eigen::Vector3d ap: vec_tran[i])
      {
        int xyz[3] = {0, 0, 0};
        for(int k = 0; k < 3; k++)
          if(ap(k) > center_turb[k])
            xyz[k] = 1;

        Eigen::Vector3d pvec(ap(0), ap(1), ap(2));
        
        int leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];
        covMats[leafnum].push(pvec);
      }
    }

    double ratios[2] = {1.0/(3.0*3.0), 2.0*2.0};
    int num_all = 0, num_qua = 0;
    for(int i = 0; i < 8; i++)
      if(covMats[i].N > 10)
      {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMats[i].cov());
        double child_eva0 = (saes.eigenvalues()[0]);
        if(child_eva0 > ratios[0]*eva0 && child_eva0 < ratios[1]*eva0)
          num_qua++;
        num_all++;
      }

    double prop = 1.0 * num_qua / num_all;

    if(prop < 0.5) return 0;
    return 1;
  }

  /**
   * @brief 将 ci 帧的点云按八叉树子区域划分到 8 个子节点
   * leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2]，xyz[k]=1 表示点在第 k 维大于体素中心
   */
  void cut_func(int ci)
  {
    PLV(3)& pvec_orig = vec_orig[ci];
    PLV(3)& pvec_tran = vec_tran[ci];

    uint a_size = pvec_tran.size();
    for(uint j = 0; j < a_size; j++)
    {
      int xyz[3] = {0, 0, 0};
      for(uint k = 0; k < 3; k++)
        if(pvec_tran[j][k] > voxel_center[k])
          xyz[k] = 1;
      int leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];
      if(leaves[leafnum] == nullptr)
      {
        leaves[leafnum] = new OCTO_TREE_NODE(win_size, eigen_thr);
        leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2*xyz[0]-1)*quater_length;
        leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2*xyz[1]-1)*quater_length;
        leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2*xyz[2]-1)*quater_length;
        leaves[leafnum]->quater_length = quater_length / 2.0;
        leaves[leafnum]->layer = layer + 1;
      }

      leaves[leafnum]->vec_orig[ci].push_back(pvec_orig[j]);
      leaves[leafnum]->vec_tran[ci].push_back(pvec_tran[j]);
      
      leaves[leafnum]->sig_orig[ci].push(pvec_orig[j]);
      leaves[leafnum]->sig_tran[ci].push(pvec_tran[j]);
    }

    PLV(3)().swap(pvec_orig);
    PLV(3)().swap(pvec_tran);
  }

  /**
   * @brief 递归细分八叉树：判定平面或继续向下分割
   * 达到 PLANE 状态时释放 vec_orig/vec_tran 以节省内存
   */
  void recut()
  {
    if(octo_state == UNKNOWN)
    {
      int point_size = 0;
      for(int i = 0; i < win_size; i++)
        point_size += sig_orig[i].N;
      
      if(point_size < MIN_PT)
      {
        octo_state = MID_NODE;
        vector<PLV(3)>().swap(vec_orig);
        vector<PLV(3)>().swap(vec_tran);
        vector<VOX_FACTOR>().swap(sig_orig);
        vector<VOX_FACTOR>().swap(sig_tran);
        return;
      }

      if(judge_eigen())
      {
        octo_state = PLANE;
        #ifndef ENABLE_FILTER
        #ifndef ENABLE_RVIZ
        vector<PLV(3)>().swap(vec_orig);
        vector<PLV(3)>().swap(vec_tran);
        #endif
        #endif
        return;
      }
      else
      {
        if(layer == layer_limit)
        {
          octo_state = MID_NODE;
          vector<PLV(3)>().swap(vec_orig);
          vector<PLV(3)>().swap(vec_tran);
          vector<VOX_FACTOR>().swap(sig_orig);
          vector<VOX_FACTOR>().swap(sig_tran);
          return;
        }
        vector<VOX_FACTOR>().swap(sig_orig);
        vector<VOX_FACTOR>().swap(sig_tran);
        for(int i = 0; i < win_size; i++)
          cut_func(i);
      }
    }
    
    for(int i = 0; i < 8; i++)
      if(leaves[i] != nullptr)
        leaves[i]->recut();
  }

  /** @brief 递归收集 PLANE 叶节点的 sig_orig，推入 vox_opt 供 BA 优化 */
  void tras_opt(VOX_HESS& vox_opt)
  {
    if(octo_state == PLANE)
      vox_opt.push_voxel(&sig_orig, &vec_orig);
    else
      for(int i = 0; i < 8; i++)
        if(leaves[i] != nullptr)
          leaves[i]->tras_opt(vox_opt);
  }

  void tras_display(int layer = 0)
  {
    float ref = 255.0*rand()/(RAND_MAX + 1.0f);
    pcl::PointXYZINormal ap;
    ap.intensity = ref;

    if(octo_state == PLANE)
    {
      // std::vector<unsigned int> colors;
			// colors.push_back(static_cast<unsigned int>(rand() % 256));
			// colors.push_back(static_cast<unsigned int>(rand() % 256));
			// colors.push_back(static_cast<unsigned int>(rand() % 256));
      pcl::PointCloud<pcl::PointXYZINormal> color_cloud;

      for(int i = 0; i < win_size; i++)
      {
        for(size_t j = 0; j < vec_tran[i].size(); j++)
        {
          Eigen::Vector3d& pvec = vec_tran[i][j];
          ap.x = pvec.x();
          ap.y = pvec.y();
          ap.z = pvec.z();
          // ap.b = colors[0];
          // ap.g = colors[1];
          // ap.r = colors[2];
          ap.normal_x = sqrt(value_vector[1] / value_vector[0]);
          ap.normal_y = sqrt(value_vector[2] / value_vector[0]);
          ap.normal_z = sqrt(value_vector[0]);
          // ap.curvature = total;
          color_cloud.push_back(ap);
        }
      }

      #ifdef ENABLE_RVIZ
      sensor_msgs::PointCloud2 dbg_msg;
      pcl::toROSMsg(color_cloud, dbg_msg);
      dbg_msg.header.frame_id = "camera_init";
      pub_residual.publish(dbg_msg);

      visualization_msgs::Marker marker;
      visualization_msgs::MarkerArray marker_array;
      marker.header.frame_id = "camera_init";
      marker.header.stamp = ros::Time::now();
      marker.ns = "basic_shapes";
      marker.id = BINGO_CNT; BINGO_CNT++;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.color.a = 1;
      marker.color.r = layer==0?1:0;
      marker.color.g = layer==1?1:0;
      marker.color.b = layer==2?1:0;
      marker.scale.x = 0.01;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.lifetime = ros::Duration();
      geometry_msgs::Point apoint;
      apoint.x = center(0); apoint.y = center(1); apoint.z = center(2);
      marker.points.push_back(apoint);
      apoint.x += 0.2*direct(0); apoint.y += 0.2*direct(1); apoint.z += 0.2*direct(2);
      marker.points.push_back(apoint);
      marker_array.markers.push_back(marker);
      pub_direct.publish(marker_array);
      #endif
    }
    else
    {
      if(layer == layer_limit) return;
      layer++;
      for(int i = 0; i < 8; i++)
        if(leaves[i] != nullptr)
          leaves[i]->tras_display(layer);
    }
  }
};

/** @brief 八叉树根节点，与 OCTO_TREE_NODE 相同，语义上表示体素地图中的一个顶层体素 */
class OCTO_TREE_ROOT: public OCTO_TREE_NODE
{
public:
  OCTO_TREE_ROOT(int _winsize, float _eigen_thr): OCTO_TREE_NODE(_winsize, _eigen_thr){}
};

/**
 * @brief 体素 BA 优化器
 *
 * 使用 Levenberg-Marquardt 阻尼迭代：
 *   (H + u*diag(H)) * δx = -J^T * r
 * 旋转更新：R ← R * Exp(δθ)，平移更新：p ← p + δt
 */
class VOX_OPTIMIZER
{
public:
  int win_size, jac_leng, imu_leng;
  VOX_OPTIMIZER(int _win_size = WIN_SIZE): win_size(_win_size)
  {
    jac_leng = DVEL * win_size;    // 6 * win_size
    imu_leng = DIM * win_size;     // 15 * win_size (保留字段)
  }

  /**
   * @brief 多线程累加 Hessian 和 Jacobian
   * @return 残差（体素最小特征值之和或均值，取决于 AVG_THR）
   */
  double divide_thread(vector<IMUST>& x_stats, VOX_HESS& voxhess, vector<IMUST>& x_ab,
                       Eigen::MatrixXd& Hess,Eigen::VectorXd& JacT)
  {
    double residual = 0;
    Hess.setZero(); JacT.setZero();
    PLM(-1) hessians(thd_num);
    PLV(-1) jacobins(thd_num);

    for(int i = 0; i < thd_num; i++)
    {
      hessians[i].resize(jac_leng, jac_leng);
      jacobins[i].resize(jac_leng);
    }

    int tthd_num = thd_num;
    vector<double> resis(tthd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < tthd_num) tthd_num = 1;

    vector<thread*> mthreads(tthd_num);
    double part = 1.0 * g_size / tthd_num;
    for(int i = 0; i < tthd_num; i++)
      mthreads[i] = new thread(&VOX_HESS::acc_evaluate2, &voxhess, x_stats, part*i, part*(i+1),
                               ref(hessians[i]), ref(jacobins[i]), ref(resis[i]));

    for(int i = 0; i < tthd_num; i++)
    {
      mthreads[i]->join();
      Hess += hessians[i];
      JacT += jacobins[i];
      residual += resis[i];
      delete mthreads[i];
    }
    #ifdef AVG_THR
    return residual/g_size;
    #else
    return residual;
    #endif
  }

  /***************************************************
  * @brief 计算残差
  * @detail 
  * @param[in] x_stats: 位姿
  * @param[in] voxhess: 
  * @param[in] x_ab: 未用
  * @param[in] is_avg: false--残差和; true--残差均值
  * @return 残差
  ****************************************************/
  double only_residual(vector<IMUST>& x_stats, VOX_HESS& voxhess, vector<IMUST>& x_ab, bool is_avg = false)
  {
    double residual2 = 0;
    voxhess.evaluate_only_residual(x_stats, residual2);
    if(is_avg) return residual2 / voxhess.plvec_voxels.size();
    return residual2;
  }

  /***************************************************
  * @brief 去除离群值
  * @detail 体素按照其点云的最小特征值由大到小排列后，删除一定比例 ratio
  * @param[in] x_stats: 位姿
  * @param[in/out] voxhess: 
  * @param[in] ratio: 离群比例
  * @return 无
  ****************************************************/
  void remove_outlier(vector<IMUST>& x_stats, VOX_HESS& voxhess, double ratio)
  {
    std::vector<double> residuals = voxhess.evaluate_residual(x_stats);
    std::sort(residuals.begin(), residuals.end()); // sort in ascending order
    double threshold = residuals[std::floor((1-ratio)*voxhess.plvec_voxels.size())-1];
    int reject_num = std::floor(ratio * voxhess.plvec_voxels.size());
    // std::cout << "vox_num before " << voxhess.plvec_voxels.size();
    // std::cout << ", reject threshold " << std::setprecision(3) << threshold << ", rejected " << reject_num;
    voxhess.remove_residual(x_stats, threshold, reject_num);
    // std::cout << ", vox_num after " << voxhess.plvec_voxels.size() << std::endl;
  }

  /**
   * @brief LM 阻尼迭代优化位姿
   *
   * @param x_stats   输入输出：滑窗位姿
   * @param voxhess   体素 Hessian 数据
   * @param residual  输出：最终残差
   * @param hess_vec  输出：Hessian 非对角块的对角近似 (供 PGO)
   * @param mem_cost  输出：优化过程内存峰值 (KB)
   */
  void damping_iter(vector<IMUST>& x_stats, VOX_HESS& voxhess, double& residual,
                    PLV(6)& hess_vec, size_t& mem_cost)
  {
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(jac_leng, jac_leng), Hess(jac_leng, jac_leng),
                    HessuD(jac_leng, jac_leng);
    Eigen::VectorXd JacT(jac_leng), dxi(jac_leng), new_dxi(jac_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    vector<IMUST> x_stats_temp;

    vector<IMUST> x_ab(win_size);  // 相邻帧相对位姿（预留，当前未使用）
    x_ab[0] = x_stats[0];
    for(int i=1; i<win_size; i++)
    {
      x_ab[i].p = x_stats[i-1].R.transpose() * (x_stats[i].p - x_stats[i-1].p);
      x_ab[i].R = x_stats[i-1].R.transpose() * x_stats[i].R;
    }

    double hesstime = 0;
    double solvtime = 0;
    size_t max_mem = 0;
    double loop_num = 0;
    for(int i = 0; i < 10; i++)
    {
      if(is_calc_hess)
      {
        double tm = ros::Time::now().toSec();
        residual1 = divide_thread(x_stats, voxhess, x_ab, Hess, JacT);
        hesstime += ros::Time::now().toSec() - tm;
      }

      double tm = ros::Time::now().toSec();
      // LM 阻尼：H_damped = H + u * diag(H)
      D.diagonal() = Hess.diagonal();
      HessuD = Hess + u*D;
      double t1 = ros::Time::now().toSec();
      // 稀疏 Cholesky 求解线性系统
      Eigen::SparseMatrix<double> A1_sparse(jac_leng, jac_leng);
      std::vector<Eigen::Triplet<double>> tripletlist;
      for(int a = 0; a < jac_leng; a++)
        for(int b = 0; b < jac_leng; b++)
          if(HessuD(a, b) != 0)
          {
            tripletlist.push_back(Eigen::Triplet<double>(a, b, HessuD(a, b)));
            //A1_sparse.insert(a, b) = HessuD(a, b);
          }
      A1_sparse.setFromTriplets(tripletlist.begin(), tripletlist.end());
      A1_sparse.makeCompressed();
      Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> Solver_sparse;
      Solver_sparse.compute(A1_sparse);
      size_t temp_mem = check_mem();
      if(temp_mem > max_mem) max_mem = temp_mem;
      dxi = Solver_sparse.solve(-JacT);
      temp_mem = check_mem();
      if(temp_mem > max_mem) max_mem = temp_mem;
      solvtime += ros::Time::now().toSec() - tm;
      // new_dxi = Solver_sparse.solve(-JacT);
      // printf("new solve time cost %f\n",ros::Time::now().toSec() - t1);
      // relative_err = ((Hess + u*D)*dxi + JacT).norm()/JacT.norm();
      // absolute_err = ((Hess + u*D)*dxi + JacT).norm();
      // std::cout<<"relative error "<<relative_err<<std::endl;
      // std::cout<<"absolute error "<<absolute_err<<std::endl;
      // std::cout<<"delta x\n"<<(new_dxi-dxi).transpose()/dxi.norm()<<std::endl;

      x_stats_temp = x_stats;
      for(int j = 0; j < win_size; j++)
      {
        x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DVEL*j, 0));
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DVEL*j+3, 0);
      }

      double q1 = 0.5*dxi.dot(u*D*dxi-JacT);
      #ifdef AVG_THR
      residual2 = only_residual(x_stats_temp, voxhess, x_ab, true);
      q1 /= voxhess.plvec_voxels.size();
      #else
      residual2 = only_residual(x_stats_temp, voxhess, x_ab);
      #endif
      residual = residual2;
      q = (residual1-residual2);
      // printf("iter%d: (%lf %lf) u: %lf v: %lf q: %lf %lf %lf\n",
      //        i, residual1, residual2, u, v, q/q1, q1, q);
      loop_num = i+1;
      // if(hesstime/loop_num > 1) printf("Avg. Hessian time: %lf ", hesstime/loop_num);
      // if(solvtime/loop_num > 1) printf("Avg. solve time: %lf\n", solvtime/loop_num);
      // if(double(max_mem/1048576.0) > 2.0) printf("Max mem: %lf\n", double(max_mem/1048576.0));
      
      // Nielsen LM 策略：接受步长则减小阻尼 u，拒绝则增大 u
      if(q > 0)
      {
        x_stats = x_stats_temp;
        q = q / q1;
        v = 2;
        q = 1 - pow(2*q-1, 3);
        u *= (q<one_three ? one_three:q);
        is_calc_hess = true;
      }
      else
      {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;  // 拒绝步长时不重算 Hessian
      }
      #ifdef AVG_THR
      if((fabs(residual1-residual2)/residual1) < 0.05 || i == 9)
      {
        if(mem_cost < max_mem) mem_cost = max_mem;
        for(int j = 0; j < win_size-1; j++)
          for(int k = j+1; k < win_size; k++)
            hess_vec.push_back(Hess.block<DVEL, DVEL>(DVEL*j, DVEL*k).diagonal().segment<DVEL>(0));
        break;
      }
      #else
      if(fabs(residual1-residual2)<1e-9) break;
      #endif
    }
  }

  /** @brief 读取当前进程 RSS 内存占用 (KB)，来自 /proc/self/status */
  size_t check_mem()
  {
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while(fgets(line, 128, file) != nullptr)
    {
      if(strncmp(line, "VmRSS:", 6) == 0)
      {
        int len = strlen(line);

        const char* p = line;
        for(; std::isdigit(*p) == false; ++p){}

        line[len - 3] = 0;
        result = atoi(p);

        break;
      }
    }
    fclose(file);

    return result;
  }
};

#endif