#include "dyros_red_controller/quadraticprogram.h"

CQuadraticProgram::CQuadraticProgram()
{
    Initialize();
}
CQuadraticProgram::~CQuadraticProgram()
{
}

void CQuadraticProgram::Initialize()
{
    _bInitialized = false;
    _num_var = 1;
    _num_cons = 1;
}

void CQuadraticProgram::InitializeProblemSize(const int &num_var, const int &num_cons)
{
    _QPprob = SQProblem(num_var, num_cons);
    _bool_constraint_Ax = false;
    _bool_constraint_x = false;
    _num_var = num_var;
    _num_cons = num_cons;
    _H.resize(_num_var, _num_var);
    _H.setZero();
    _g.resize(_num_var);
    _g.setZero();
    _A.resize(_num_cons, _num_var);
    _A.setZero();
    _lbA.resize(_num_cons);
    _lbA.setZero();
    _ubA.resize(_num_cons);
    _ubA.setZero();
    _lb.resize(_num_var);
    _lb.setZero();
    _ub.resize(_num_var);
    _ub.setZero();
    _bInitialized = false;
}

void CQuadraticProgram::UpdateMinProblem(const MatrixXd &H, const VectorXd &g)
{
    _H = H;
    _g = g;
}

void CQuadraticProgram::UpdateSubjectToAx(const MatrixXd &A, const VectorXd &lbA, const VectorXd &ubA)
{
    _A = A;
    _lbA = lbA;
    _ubA = ubA;
    _bool_constraint_Ax = true;
}

void CQuadraticProgram::UpdateSubjectToX(const VectorXd &lb, const VectorXd &ub)
{
    _lb = lb;
    _ub = ub;
    _bool_constraint_x = true;
}

void CQuadraticProgram::DeleteSubjectToAx()
{
    _bool_constraint_Ax = false;
}

void CQuadraticProgram::DeleteSubjectToX()
{
    _bool_constraint_x = false;
}

void CQuadraticProgram::PrintMinProb()
{
    cout << "------------------------------------------------------------------------------" << endl;
    cout << "----------------------------------    H    -----------------------------------" << endl;
    cout << "------------------------------------------------------------------------------" << endl;
    cout << _H << endl;
    cout << "------------------------------------------------------------------------------" << endl;
    cout << "----------------------------------    g    -----------------------------------" << endl;
    cout << "------------------------------------------------------------------------------" << endl;
    cout << _g.transpose() << endl;
    cout << "------------------------------------------------------------------------------" << endl;
}

void CQuadraticProgram::PrintSubjectToAx()
{
    if (_bool_constraint_Ax == true)
    {
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "----------------------------------    A    -----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _A << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "---------------------------------    lbA    ----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _lbA.transpose() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "---------------------------------    ubA    ----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _ubA.transpose() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
    }
    else
    {
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "                   s.t. lbA <= Ax <= ubA is not inserted.                     " << endl;
        cout << "------------------------------------------------------------------------------" << endl;
    }
}

void CQuadraticProgram::PrintSubjectTox()
{
    if (_bool_constraint_x == true)
    {
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "---------------------------------    lb    -----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _lb.transpose() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "---------------------------------    ub    -----------------------------------" << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        cout << _ub.transpose() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
    }
    else
    {
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "                     s.t. lb <= x <= ub is not inserted.                      " << endl;
        cout << "------------------------------------------------------------------------------" << endl;
    }
}

void CQuadraticProgram::EnableEqualityCondition(const double Tolerance)
{
    _options.enableEqualities = BT_TRUE;
    real_t Tolerance_equal = Tolerance;
    _options.boundRelaxation = Tolerance_equal;
}

void CQuadraticProgram::DisableEqualityCondition()
{
    _options.enableEqualities = BT_FALSE;
}

VectorXd CQuadraticProgram::SolveQPoases(const int &num_max_iter)
{
    //translate eigen to real_t formulation
    real_t H_realt[_num_var * _num_var]; // H in min eq 1/2x'Hx + x'g
    for (int i = 0; i < _num_var; i++)
    {
        for (int j = 0; j < _num_var; j++)
        {
            H_realt[_num_var * j + i] = _H(j, i);
        }
    }

    real_t g_realt[_num_var]; // g in min eq 1/2x'Hx + x'g
    for (int i = 0; i < _num_var; i++)
    {
        g_realt[i] = _g(i);
    }

    real_t A_realt[_num_cons * _num_var]; // A in s.t. eq lbA<= Ax <=ubA
    if (_bool_constraint_Ax == true)
    {
        for (int i = 0; i < _num_var; i++)
        {
            for (int j = 0; j < _num_cons; j++)
            {
                A_realt[_num_var * j + i] = _A(j, i);
            }
        }
    }

    real_t lbA_realt[_num_cons]; // lbA in s.t. eq lbA<= Ax <=ubA
    if (_bool_constraint_Ax == true)
    {
        for (int i = 0; i < _num_cons; i++)
        {
            lbA_realt[i] = _lbA(i);
        }
    }

    real_t ubA_realt[_num_cons]; // ubA in s.t. eq lbA<= Ax <=ubA
    if (_bool_constraint_Ax == true)
    {
        for (int i = 0; i < _num_cons; i++)
        {
            ubA_realt[i] = _ubA(i);
        }
    }
    real_t lb_realt[_num_var]; //lb in s.t. eq lb <= x <= ub
    if (_bool_constraint_x == true)
    {
        for (int i = 0; i < _num_var; i++)
        {
            lb_realt[i] = _lb(i);
        }
    }

    real_t ub_realt[_num_var]; //ub in s.t. eq lb <= x <= ub
    if (_bool_constraint_x == true)
    {
        for (int i = 0; i < _num_var; i++)
        {
            ub_realt[i] = _ub(i);
        }
    }
    int_t nWSR = num_max_iter;

    _options.printLevel = PL_NONE;
    //_options.printLevel = PL_DEBUG_ITER;
    _QPprob.setOptions(_options);

    returnValue m_status;
    if (_bInitialized == false) //init
    {
        if (_bool_constraint_Ax == true && _bool_constraint_x == true)
        {
            m_status = _QPprob.init(H_realt, g_realt, A_realt, lb_realt, ub_realt, lbA_realt, ubA_realt, nWSR);
        }
        else if (_bool_constraint_Ax == true && _bool_constraint_x == false)
        {
            m_status = _QPprob.init(H_realt, g_realt, A_realt, 0, 0, lbA_realt, ubA_realt, nWSR);
        }
        else if (_bool_constraint_Ax == false && _bool_constraint_x == true)
        {
            m_status = _QPprob.init(H_realt, g_realt, 0, lb_realt, ub_realt, 0, 0, nWSR);
        }
        else
        {
            m_status = _QPprob.init(H_realt, g_realt, 0, 0, 0, 0, 0, nWSR);
        }
        _bInitialized = true;
    }
    else //hotstart
    {
        if (_bool_constraint_Ax == true && _bool_constraint_x == true)
        {
            m_status = _QPprob.hotstart(H_realt, g_realt, A_realt, lb_realt, ub_realt, lbA_realt, ubA_realt, nWSR);
        }
        else if (_bool_constraint_Ax == true && _bool_constraint_x == false)
        {
            m_status = _QPprob.hotstart(H_realt, g_realt, A_realt, 0, 0, lbA_realt, ubA_realt, nWSR);
        }
        else if (_bool_constraint_Ax == false && _bool_constraint_x == true)
        {
            m_status = _QPprob.hotstart(H_realt, g_realt, 0, lb_realt, ub_realt, 0, 0, nWSR);
        }
        else
        {
            m_status = _QPprob.hotstart(H_realt, g_realt, 0, 0, 0, 0, 0, nWSR);
        }
    }

    real_t Xopt_realt[_num_var];
    _QPprob.getPrimalSolution(Xopt_realt);

    VectorXd Xopt(_num_var);
    for (int i = 0; i < _num_var; i++)
    {
        Xopt(i) = Xopt_realt[i];
    }

    return Xopt;
}
