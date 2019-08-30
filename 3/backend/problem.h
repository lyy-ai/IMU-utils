#ifndef MYSLAM_BACKEND_PROBLEM_H
#define MYSLAM_BACKEND_PROBLEM_H

#include <unordered_map>
#include <map>
#include <memory>

#include "backend/eigen_types.h"
#include "backend/edge.h"
#include "backend/vertex.h"

typedef unsigned long ulong;

namespace myslam
 {
    namespace backend 
    {

    class Problem 
    {
    public:

    /**
     * 问题的类型
     * SLAM问题还是通用的问题
     *
     * 如果是SLAM问题那么pose和landmark是区分开的，Hessian以稀疏方式存储
     * SLAM问题只接受一些特定的Vertex和Edge
     * 如果是通用问题那么hessian是稠密的，除非用户设定某些vertex为marginalized
     */
    enum class ProblemType 
    {
        SLAM_PROBLEM,
        GENERIC_PROBLEM
    };
    typedef unsigned long ulong;
//    typedef std::unordered_map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
    typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
    typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
    typedef std::unordered_multimap<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Problem(ProblemType problemType);

    ~Problem();

    bool AddVertex(std::shared_ptr<Vertex> vertex);

    /**
     * remove a vertex
     * @param vertex_to_remove
     */
    bool RemoveVertex(std::shared_ptr<Vertex> vertex);

    bool AddEdge(std::shared_ptr<Edge> edge);

    bool RemoveEdge(std::shared_ptr<Edge> edge);

    /**
     * 求解此问题
     * @param iterations
     * @return
     */
    bool Solve(int iterations);



private:

   
    /// 设置各顶点的ordering_index
    void SetOrdering();

    /// 构造大H矩阵
    void MakeHessian();

    /// 解线性方程
    void SolveLinearSystem();

    /// 更新状态变量
    void UpdateStates();

    void RollbackStates(); // 有时候 update 后残差会变大，需要退回去，重来

    /// Levenberg
    /// 计算LM算法的初始Lambda
    void ComputeLambdaInitLM();

    /// Hessian 对角线加上或者减去  Lambda
    void AddLambdatoHessianLM();

    void RemoveLambdaHessianLM();

    /// LM 算法中用于判断 Lambda 在上次迭代中是否可以，以及Lambda怎么缩放
    bool IsGoodStepInLM();
;

    double currentLambda_;
    double currentChi_;//迭代次数
    double stopThresholdLM_;    // LM 迭代退出阈值条件
    double ni_;                 //控制 Lambda 缩放大小

    ProblemType problemType_;

    /// 整个信息矩阵
    MatXX Hessian_;
    VecX b_;
    VecX delta_x_;

    /// 先验部分信息
    MatXX H_prior_;
    VecX b_prior_;
    MatXX Jt_prior_inv_;
    VecX err_prior_;

  

    /// all vertices
    HashVertex verticies_;

    /// all edges
    HashEdge edges_;

    /// 由vertex id查询edge
    HashVertexIdToEdge vertexToEdge_;

    /// Ordering related
    ulong ordering_poses_ = 0;
    ulong ordering_landmarks_ = 0;
    ulong ordering_generic_ = 0;

     // verticies need to marg. <Ordering_id_, Vertex>
    HashVertex verticies_marg_;
    
     double t_hessian_cost_ = 0.0;
};

}
}

#endif
