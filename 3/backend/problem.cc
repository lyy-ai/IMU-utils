#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
#include "backend/problem.h"
#include "utils/tic_toc.h"

#ifdef USE_OPENMP

#include <omp.h>

#endif

using namespace std;

namespace myslam
{
    namespace backend
    {
        Problem::Problem(ProblemType problemType):problemType_(problemType)
        {
            verticies_marg_.clear();
        }

        Problem::~Problem(){}

        bool Problem::AddVertex(std::shared_ptr<Vertex> vertex)
        {   
            if(verticies_.find(vertex->Id())!=verticies_.end())
            {
                   return false;//顶点之前被加入
            }
            else
            {
                verticies_.insert(pair<unsigned long,shared_ptr<Vertex>>(vertex->Id(),vertex));

            }
            return true;
              
        }

        bool Problem::AddEdge(std::shared_ptr<Edge> edge)
        {   
            if(edges_.find(edge->Id())!=edges_.end())
            {
                   return false;//顶点之前被加入
            }
            else
            {
                edges_.insert(pair<unsigned long,shared_ptr<Edge>>(edge->Id(),edge));


            }
            for (auto &vertex: edge->Verticies()) 
            {
                 vertexToEdge_.insert(pair<ulong, shared_ptr<Edge>>(vertex->Id(), edge));
            }
            return true;
              
        }

        bool Problem::Solve(int iterations)
        {
            if(edges_.size()==0 || verticies_.size()==0)
            {
                std::cerr<<" cannot solve problem without edges or verticies"<<std::endl;
                return false;
            }

            TicToc t_solve;

            //统计优化变量的维数，为构建H矩阵做准备
            SetOrdering();

            //遍历边，构建H矩阵
            MakeHessian();
                // LM 初始化
             ComputeLambdaInitLM();
            // LM 算法迭代求解
            bool stop = false;
             int iter = 0;
                 while (!stop && (iter < iterations))
                  {
                std::cout << "iter: " << iter << " , chi= " << currentChi_ << " , Lambda= " << currentLambda_
                  << std::endl;
            bool oneStepSuccess = false;
            int false_cnt = 0;
         while (!oneStepSuccess)  // 不断尝试 Lambda, 直到成功迭代一步
        {
            // setLambda
            AddLambdatoHessianLM();
            // 第四步，解线性方程 H X = B
            SolveLinearSystem();
            //
            RemoveLambdaHessianLM();

            // 优化退出条件1： delta_x_ 很小则退出
            if (delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10) {
                stop = true;
                break;
            }

            // 更新状态量 X = X+ delta_x
            UpdateStates();
            // 判断当前步是否可行以及 LM 的 lambda 怎么更新
            oneStepSuccess = IsGoodStepInLM();
            // 后续处理，
            if (oneStepSuccess) 
            {
                // 在新线性化点 构建 hessian
                MakeHessian();
                false_cnt = 0;
            } 
            else 
            {
                false_cnt++;
                RollbackStates();   // 误差没下降，回滚
            }
        }
        iter++;

        // 优化退出条件3： currentChi_ 跟第一次的chi2相比，下降了 1e6 倍则退出
        if (sqrt(currentChi_) <= stopThresholdLM_)
            stop = true;
         }
          std::cout << "problem solve cost: " << t_solve.toc() << " ms" << std::endl;
            std::cout << "   makeHessian cost: " << t_hessian_cost_ << " ms" << std::endl;
          return   true;

    }

        void Problem::SetOrdering() 
        {

            // 每次重新计数
             ordering_poses_ = 0;
             ordering_generic_ = 0;
            ordering_landmarks_ = 0;

        // Note:: verticies_ 是 map 类型的, 顺序是按照 id 号排序的
        // 统计带估计的所有变量的总维度
        for (auto vertex: verticies_) 
         {
             ordering_generic_ += vertex.second->LocalDimension();  // 所有的优化变量总维数
          }
        }

        void Problem::MakeHessian()
         {
            TicToc t_h;
            // 直接构造大的 H 矩阵
         ulong size = ordering_generic_;
         MatXX H(MatXX::Zero(size, size));
         VecX b(VecX::Zero(size));

        // 遍历每个残差，并计算他们的雅克比，得到最后的 H = J^T * J
        for (auto &edge: edges_) 
        {

        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();

        auto jacobians = edge.second->Jacobians();
        auto verticies = edge.second->Verticies();
        assert(jacobians.size() == verticies.size());
        for (size_t i = 0; i < verticies.size(); ++i) 
        {
            auto v_i = verticies[i];
            if (v_i->IsFixed()) continue;    // Hessian 里不需要添加它的信息，也就是它的雅克比为 0

            auto jacobian_i = jacobians[i];
            ulong index_i = v_i->OrderingId();
            ulong dim_i = v_i->LocalDimension();

            MatXX JtW = jacobian_i.transpose() * edge.second->Information();
            for (size_t j = i; j < verticies.size(); ++j) 
            {
                auto v_j = verticies[j];

                if (v_j->IsFixed()) continue;

                auto jacobian_j = jacobians[j];
                ulong index_j = v_j->OrderingId();
                ulong dim_j = v_j->LocalDimension();

                assert(v_j->OrderingId() != -1);
                MatXX hessian = JtW * jacobian_j;
                // 所有的信息矩阵叠加起来
                H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
                if (j != i) 
                {
                    // 对称的下三角
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                }
            }
            b.segment(index_i, dim_i).noalias() -= JtW * edge.second->Residual();
          }

       }
    Hessian_ = H;
    b_ = b;
    t_hessian_cost_ += t_h.toc();

    delta_x_ = VecX::Zero(size);  // initial delta_x = 0_n;

    }
   
   /// LM
   void Problem::ComputeLambdaInitLM() 
   {
    ni_ = 2.;
    currentLambda_ = -1.;
    currentChi_ = 0.0;
    // TODO:: robust cost chi2
    for (auto edge: edges_) 
    {
        currentChi_ += edge.second->Chi2();
    }
    if (err_prior_.rows() > 0)
        currentChi_ += err_prior_.norm();

    stopThresholdLM_ = 1e-6 * currentChi_;          // 迭代条件为 误差下降 1e-6 倍

    double maxDiagonal = 0;
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < size; ++i) 
    {
        maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
    }
    double tau = 1e-5;
    currentLambda_ = tau * maxDiagonal;
   }


   void Problem::AddLambdatoHessianLM() 
   {
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < size; ++i)
    {
        Hessian_(i, i) += currentLambda_;
    }
  }

  void Problem::SolveLinearSystem() 
  {

        delta_x_ = Hessian_.inverse() * b_;
        // delta_x_ = H.ldlt().solve(b_);

  }
  void Problem::RemoveLambdaHessianLM()
   {
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    // TODO:: 这里不应该减去一个，数值的反复加减容易造成数值精度出问题？而应该保存叠加lambda前的值，在这里直接赋值
    for (ulong i = 0; i < size; ++i) {
        Hessian_(i, i) -= currentLambda_;
    }
  }

  void Problem::UpdateStates() 
  {
    for (auto vertex: verticies_) {
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);

        // 所有的参数 x 叠加一个增量  x_{k+1} = x_{k} + delta_x
        vertex.second->Plus(delta);
    }
    }

    void Problem::RollbackStates()
    {
    for (auto vertex: verticies_) 
    {
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x_.segment(idx, dim);

        // 之前的增量加了后使得损失函数增加了，我们应该不要这次迭代结果，所以把之前加上的量减去。
        vertex.second->Plus(-delta);
    }
    }

    bool Problem::IsGoodStepInLM() 
    {
    double scale = 0;
    scale = delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
    scale += 1e-3;    // make sure it's non-zero :)

    // recompute residuals after update state
    // 统计所有的残差
    double tempChi = 0.0;
    for (auto edge: edges_) 
    {
        edge.second->ComputeResidual();
        tempChi += edge.second->Chi2();
    }

    double rho = (currentChi_ - tempChi) / scale;
    if (rho > 0 && isfinite(tempChi))   // last step was good, 误差在下降
    {
        double alpha = 1. - pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        currentLambda_ *= scaleFactor;
        ni_ = 2;
        currentChi_ = tempChi;
        return true;
    } 
    else
     {
        currentLambda_ *= ni_;
        ni_ *= 2;
        return false;
    }
   }


    }
}
