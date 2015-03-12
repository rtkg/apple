#include <hqp_controllers/hqp.h>

namespace hqp_controllers {

//-----------------------------------------
std::ostream& operator<<(std::ostream& str, HQPStage const& stage)
{
    str<<"HQPSTAGE"<<std::endl;
    str<<"solved: "<<stage.solved_<<std::endl;
    str<<"s_dim_: "<<stage.s_dim_<<std::endl;
    str<<"de_: "<<stage.de_.transpose()<<std::endl;
    str<<"is_equalities_: ";
    for(unsigned int i=0; i<stage.is_equalities_.size();i++)
        str<<stage.is_equalities_.at(i)<<" ";

    str<<std::endl<<"J_:"<<std::endl<<stage.J_<<std::endl;
    str<<"x_: "<<stage.x_.transpose()<<std::endl;
    str<<"w_: "<<stage.w_.transpose()<<std::endl;
}
//-----------------------------------------
HQPStage::HQPStage(Task const& task) : solved_(false)
{
    de_ = task.getTaskVelocity();
    s_dim_ = de_.rows();
    J_ = task.getTaskJacobian();
    is_equalities_.resize(s_dim_);
    std::fill(is_equalities_.begin(), is_equalities_.end(), task.getIsEqualityTask());

    x_.resize(J_.cols());
    x_.setZero();
    w_.resize(s_dim_);
    w_.setZero();
}
//-----------------------------------------
void HQPStage::appendTask(Task const& task)
{
    unsigned int t_dim = task.getTaskFunction().rows();
    de_.conservativeResize(s_dim_ + t_dim);
    de_.tail(t_dim) = task.getTaskVelocity();
    J_.conservativeResize(s_dim_ + t_dim, Eigen::NoChange);
    J_.bottomRows(t_dim) = task.getTaskJacobian();

    for(unsigned int i=0; i<t_dim; i++)
        is_equalities_.push_back(task.getIsEqualityTask());

    w_.resize(s_dim_ + t_dim);
    w_.setZero();

    s_dim_ += t_dim;
    solved_ = false;
}
//-----------------------------------------
HQPSolver::HQPSolver()
{
    env_.set(GRB_IntParam_OutputFlag, OUTPUT_FLAG);
    env_.set(GRB_IntParam_Presolve, PRESOLVE);
    env_.set(GRB_DoubleParam_OptimalityTol, OPTIMALITY_TOL);
    env_.set(GRB_IntParam_ScaleFlag, SCALE_FLAG);
    env_.set(GRB_DoubleParam_TimeLimit, TIME_LIMIT);
    env_.set(GRB_IntParam_DualReductions, DUAL_REDUCTIONS);
}
//-----------------------------------------
void HQPSolver::reset()
{
    b_.resize(0);
    w_.resize(0);
    A_.resize(0,0);
    senses_.clear();
}
//-----------------------------------------
bool HQPSolver::solve(std::map<unsigned int, boost::shared_ptr<HQPStage> > &hqp)
{
    if(hqp.empty())
        return false;

    reset();

    std::map<unsigned int, boost::shared_ptr<HQPStage> >::const_iterator it = hqp.begin();
    unsigned int x_dim = it->second->J_.cols();
    A_.resize(Eigen::NoChange, x_dim);

    try
    {
        //iterate through all stages
        unsigned int s_count = 0; //stage counter
        for (it; it!=hqp.end(); ++it)
        {
            ROS_ASSERT(it->second->J_.cols() == x_dim); //make sure the stage jacobian column dimensions are consistent
            s_count++;
            GRBModel model(env_);
            unsigned int s_dim = it->second->s_dim_; //dimension of the current stage
            unsigned int s_acc_dim = b_.rows(); //accumulated dimensions of all the previously solved stages

            //append the new signs, jacobian matrix and task velocity vector to the previous ones
            for(unsigned int i = 0; i<s_dim; i++)
            {
                if (it->second->is_equalities_.at(i))
                    senses_.push_back(GRB_EQUAL);
                else
                    senses_.push_back(GRB_LESS_EQUAL);
            }
            b_.conservativeResize(s_acc_dim + s_dim);
            b_.tail(s_dim) = it->second->de_;
            A_.conservativeResize(s_acc_dim + s_dim, Eigen::NoChange);
            A_.bottomRows(s_dim) = it->second->J_;
            w_.conservativeResize(s_acc_dim + s_dim);
            w_.tail(s_dim).setZero(); //set zero for now - will be filled with the solution later

            //at each iteration, variables are x (the joint velocities) + the slack variables
            double* lb_x = new double[x_dim]; std::fill_n(lb_x, x_dim, -GRB_INFINITY);
            double* ub_x = new double[x_dim]; std::fill_n(ub_x, x_dim, GRB_INFINITY);
            GRBVar* x =model.addVars(lb_x, ub_x, NULL, NULL, NULL, x_dim);

            double* lb_w = new double[s_dim]; std::fill_n(lb_w, s_dim, -GRB_INFINITY);
            double* ub_w = new double[s_dim]; std::fill_n(ub_w, s_dim, GRB_INFINITY);
            GRBVar* w =model.addVars(lb_w, ub_w, NULL, NULL, NULL, s_dim); //slack variables
            model.update();

            GRBLinExpr* lhsides = new GRBLinExpr[s_dim + s_acc_dim];
           // char* senses = new char[s_dim + s_acc_dim];
            double* rhsides = new double[s_dim + s_acc_dim];
            double* coeff_x = new double[x_dim];
            double* coeff_w = new double[s_dim];

            //========== CREATE GUROBI EXPRESSIONS ===================

            //Fill in the senses array
//            for(unsigned int i=0; i<s_dim + s_acc_dim; i++)
//                senses[i] = senses_[i];

//                if(signs_[i] == "=")
//                    senses[i] = GRB_EQUAL;
//                else if (signs_[i] == "<=")
//                    senses[i] = GRB_LESS_EQUAL;
//                else if (signs_[i] == ">=")
//                    senses[i] = GRB_GREATER_EQUAL;
//                else
//                {
//                    ROS_ERROR("HQPSolver::solve(): %s is an invalid sign.",it->second->signs_->at(i).c_str());
//                    ROS_BREAK();
//                }

            //Fill in the constant right-hand side array
            Eigen::Map<Eigen::VectorXd>(rhsides, s_acc_dim + s_dim) = b_ + w_;

            //Fill in the left-hand side expressions
            for(unsigned int i=0; i<s_acc_dim; i++)
            {
                Eigen::Map<Eigen::VectorXd>(coeff_x, x_dim) = A_.row(i);
                lhsides[i].addTerms(coeff_x, x, x_dim);
            }
            for(unsigned int i=0; i<s_dim; i++)
            {
                Eigen::Map<Eigen::VectorXd>(coeff_x, x_dim) = A_.row(s_acc_dim+i);
                lhsides[s_acc_dim+i].addTerms(coeff_x, x, x_dim);
                if(s_count == 1)
                    lhsides[s_acc_dim+i] -= w[i]*0.0; //force the slack variables to be zero in the highest stage;
                else
                    lhsides[s_acc_dim+i] -= w[i];
            }

            //add constraints
            GRBConstr* constrs=model.addConstrs(lhsides,&senses_[0],rhsides,NULL,s_acc_dim + s_dim);

            //add objective
            GRBQuadExpr obj;
            std::fill_n(coeff_x, x_dim, TIKHONOV_FACTOR);
            std::fill_n(coeff_w, s_dim, 1.0);
            obj.addTerms(coeff_x, x, x, x_dim);
            obj.addTerms(coeff_w, w, w, s_dim);
            model.setObjective(obj, GRB_MINIMIZE);
            model.update();

//            //========== SOLVE ===================
            model.optimize();
            int status = model.get(GRB_IntAttr_Status);
            double runtime = model.get(GRB_DoubleAttr_Runtime);

            if (status != GRB_OPTIMAL)
            {
                if(status == GRB_TIME_LIMIT)
                    ROS_WARN("Stage solving runtime %f sec exceeds the set time limit of %f sec.", runtime, TIME_LIMIT);
                else
                    ROS_ERROR("In HQPSolver::solve(...): No optimal solution found for stage %d. Status is %d.", s_count, status);

                delete[] lb_x;
                delete[] ub_x;
                delete[] lb_w;
                delete[] ub_w;
                delete[] x;
                delete[] w;
                delete[] lhsides;
//                delete[] senses;
                delete[] rhsides;
                delete[] coeff_x;
                delete[] coeff_w;

//                //model.write("/home/rkg/Desktop/model.lp");
//                //model.write("/home/rkg/Desktop/model.sol");

                return false;
            }

            try
            {
                //Update the solution and put it in the current stage
                for(unsigned int i=0; i<x_dim; i++)
                    it->second->x_(i) = x[i].get(GRB_DoubleAttr_X);

                for(unsigned int i=0; i<s_dim; i++)
                {
                    w_(s_acc_dim + i) = w[i].get(GRB_DoubleAttr_X);
                    it->second->w_(i) = w_(s_acc_dim + i);
                }
            }
            catch(GRBException e)
            {
                ROS_ERROR("In HQPSolver::solve(...): Gurobi exception with error code %d, and error message %s when trying to extract the solution variables.", e.getErrorCode(), e.getMessage().c_str());
                //model.write("/home/rkg/Desktop/model.lp");
                //model.write("/home/rkg/Desktop/model.sol");

                return false;
            }

            it->second->solved_ = true;

////            if(s_count == hqp.size())
////            {
////                model.write("/home/rkg/Desktop/model.lp");
////                model.write("/home/rkg/Desktop/model.sol");
////            }
//            //std::cout<<"SOLVED STAGE: "<<it->first<<std::endl;
//            //std::cout<< *it->second<<std::endl;
//            //std::cout<<"runtime: "<<runtime<<std::endl;

            delete[] lb_x;
            delete[] ub_x;
            delete[] lb_w;
            delete[] ub_w;
            delete[] x;
            delete[] w;
            delete[] lhsides;
//            delete[] senses;
            delete[] rhsides;
            delete[] coeff_x;
            delete[] coeff_w;
        }
    }
    catch(GRBException e)
    {
        ROS_ERROR("In HQPSolver::solve(...): Gurobi exception with error code %d, and error message %s.", e.getErrorCode(), e.getMessage().c_str());
        return false;
    }

    //std::cerr<<"Solved HQP! Yay!"<<std::endl;
    return true;
}
//-----------------------------------------
}//end namespace hqp_controllers
