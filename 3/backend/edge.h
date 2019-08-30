#ifndef MYSLAM_BACKEND_EDGE_H
#define MYSLAM_BACKEND_EDGE_H

#include <memory>
#include <string>
#include "backend/eigen_types.h"

namespace myslam 
{
    namespace backend 
    {

    class Vertex;
    
/**
 * 边负责计算残差，残差是 预测-观测，维度在构造函数中定义
 * 代价函数是 残差*信息*残差，是一个数值，由后端求和后最小化
 */
    class Edge 
    {
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * 构造函数，会自动化配雅可比的空间
     * @param residual_dimension 残差维度
     * @param num_verticies 顶点数量
     * @param verticies_types 顶点类型名称，可以不给，不给的话check中不会检查
     */
    explicit Edge(int residual_dimension, int num_verticies,
                  const std::vector<std::string> &verticies_types = std::vector<std::string>());

    virtual ~Edge();

    
    /// 返回id
    unsigned long Id() const { return id_; }
    //设置一个顶点
    bool AddVertex(std::shared_ptr<Vertex> vertex)
    {
        verticies_.emplace_back(vertex);
        return true;
    }

    //设置一些顶点
    bool SetVertex(const std::vector<std::shared_ptr<Vertex>> &vertices)
    {
        verticies_=vertices;
        return true;
    }

        /// 返回第i个顶点
    std::shared_ptr<Vertex> GetVertex(int i) {
        return verticies_[i];
    }

    /// 返回所有顶点
    std::vector<std::shared_ptr<Vertex>> Verticies() const {
        return verticies_;
    }

    /// 计算残差，由子类实现
    virtual void ComputeResidual() = 0;

    /// 计算雅可比，由子类实现
    /// 本后端不支持自动求导，需要实现每个子类的雅可比计算方法
    virtual void ComputeJacobians() = 0;

     /// 返回信息矩阵
    MatXX Information() const {
        return information_;
    }

     /// 返回残差
    VecX Residual() const { return residual_; }

    /// 返回雅可比
    std::vector<MatXX> Jacobians() const { return jacobians_; }


   int OrderingId() const { return ordering_id_; }
    /// 计算平方误差，会乘以信息矩阵
    double Chi2();

    protected:
           unsigned long id_;  // edge id
            int ordering_id_;   //edge id in problem
           std::vector<std::string> verticies_types_;  // 各顶点类型信息，用于debug
            std::vector<std::shared_ptr<Vertex>> verticies_; // 该边对应的顶点
            VecX residual_;                 // 残差
           std::vector<MatXX> jacobians_;  // 雅可比，每个雅可比维度是 residual x vertex[i]
           MatXX information_;             // 信息矩阵

    };
    }
}
#endif