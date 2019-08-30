#include "backend/vertex.h"
#include "backend/edge.h"
#include <iostream>

using namespace std;

namespace myslam 
{
    namespace backend 
  {

    unsigned long global_edge_id = 0;

    Edge::Edge(int residual_dimension,int num_verticies,
    const std::vector<std::string> &verticies_types)
    {
        residual_.resize(residual_dimension,1);
        if(!verticies_types.empty())
            verticies_types_=verticies_types;
        jacobians_.resize(num_verticies);
        id_=global_edge_id++;

        Eigen::MatrixXd information(residual_dimension,residual_dimension);
        information.setIdentity();
        information_=information;
    }

    Edge::~Edge(){}

    double Edge::Chi2()
    {
        return residual_.transpose()*information_*residual_;
    }
  }
}