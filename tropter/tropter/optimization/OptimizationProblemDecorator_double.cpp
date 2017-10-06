#include "OptimizationProblem.h"
#include <ColPack/ColPackHeaders.h>

//#if defined(TROPTER_WITH_OPENMP) && _OPENMP
//    // TODO only include ifdef _OPENMP
//    #include <omp.h>
//    #if defined(__GNUC__)
//        #pragma GCC diagnostic push
//        #pragma GCC diagnostic ignored "-Wunknown-pragmas"
//    #elif defined(_MSC_VER)
//        #pragma warning(push)
//        #pragma warning(disable: 4068) // Disable unknown pragma warnings.
//    #endif
//#endif

using Eigen::VectorXd;

namespace tropter {

// We must implement the destructor in a context where ColPack's coloring
// class is complete (since it's used in a unique ptr member variable.).
OptimizationProblem<double>::Decorator::~Decorator() {}

OptimizationProblem<double>::Decorator::Decorator(
        const OptimizationProblem<double>& problem) :
        OptimizationProblemDecorator(problem), m_problem(problem) {}

void OptimizationProblem<double>::Decorator::
calc_sparsity(const Eigen::VectorXd& x,
        std::vector<unsigned int>& jacobian_row_indices,
        std::vector<unsigned int>& jacobian_col_indices,
        std::vector<unsigned int>& hessian_row_indices,
        std::vector<unsigned int>& hessian_col_indices) const
{
    const auto num_vars = get_num_variables();

    // Gradient.
    // =========
    // Determine the indicies of the variables used in the objective function
    // (conservative estimate of the indicies of the gradient that are nonzero).
    m_x_working = VectorXd::Zero(num_vars);
    double obj_value;
    for (int j = 0; j < (int)num_vars; ++j) {
        obj_value = 0;
        m_x_working[j] = std::numeric_limits<double>::quiet_NaN();
        m_problem.calc_objective(m_x_working, obj_value);
        m_x_working[j] = 0;
        if (std::isnan(obj_value)) {
            m_gradient_nonzero_indices.push_back(j);
        }
    }

    // Jacobian.
    // =========
    const auto num_jac_rows = get_num_constraints();

    // Determine the sparsity pattern.
    // -------------------------------
    // We do this by setting an element of x to NaN, and examining which
    // constraint equations end up as NaN (and therefore depend on that
    // element of x).
    m_x_working.setZero();
    VectorXd constr_working(num_jac_rows);
    // Initially, we store the sparsity structure in ADOL-C's compressed row
    // format, since this is what ColPack accepts.
    // This format, as described in the ADOL-C manual, is a 2-Dish array.
    // The length of the first dimension is the number of rows in the Jacobian.
    // The first element of each row is the number of nonzeros in that row of
    // the Jacobian. The remaining elements are the column indices of those
    // nonzeros. The length of each row (the second dimension) is
    // num_nonzeros_in_the_row + 1.
    std::vector<std::vector<unsigned int>> jacobian_sparsity_temp(num_jac_rows);
    size_t num_jacobian_nonzeros = 0;
    for (int j = 0; j < (int)num_vars; ++j) {
        constr_working.setZero();
        m_x_working[j] = std::numeric_limits<double>::quiet_NaN();
        m_problem.calc_constraints(m_x_working, constr_working);
        m_x_working[j] = 0;
        for (int i = 0; i < (int)num_jac_rows; ++i) {
            if (std::isnan(constr_working[i])) {
                jacobian_sparsity_temp[i].push_back(j);
                ++num_jacobian_nonzeros;
            }
        }
    }

    // Store the sparsity pattern in a raw C array, since this is what
    // ColPack requires.
    // Create a lambda that deletes the 2D C array.
    auto unsigned_int_2d_deleter = [num_jac_rows](unsigned** x) {
        std::for_each(x, x + num_jac_rows, std::default_delete<unsigned[]>());
        delete [] x;
    };
    m_jacobian_pattern_ADOLC_format = UnsignedInt2DPtr(
            new unsigned*[num_jac_rows], unsigned_int_2d_deleter);
    for (int i = 0; i < (int)num_jac_rows; ++i) {
        const auto& col_idx_for_nonzeros = jacobian_sparsity_temp[i];
        const auto num_nonzeros_this_row = col_idx_for_nonzeros.size();
        m_jacobian_pattern_ADOLC_format[i] =
                new unsigned[num_nonzeros_this_row+1];
        m_jacobian_pattern_ADOLC_format[i][0] = (unsigned)num_nonzeros_this_row;
        std::copy(col_idx_for_nonzeros.begin(), col_idx_for_nonzeros.end(),
                // Skip over the first element.
                m_jacobian_pattern_ADOLC_format[i] + 1);
    }

    // Determine the efficient perturbation directions.
    // ------------------------------------------------
    m_jacobian_coloring.reset(
            new ColPack::BipartiteGraphPartialColoringInterface(
                    SRC_MEM_ADOLC, // We're using the ADOLC sparsity format.
                    m_jacobian_pattern_ADOLC_format.get(), // Sparsity.
                    num_jac_rows, num_vars));

    // ColPack will allocate and store the seed matrix in jacobian_seed_raw.
    double** jacobian_seed_raw = nullptr;
    int jacobian_seed_num_rows; // Should be num_vars.
    int jacobian_seed_num_cols; // Number of seeds.
    m_jacobian_coloring->GenerateSeedJacobian_unmanaged(&jacobian_seed_raw,
            &jacobian_seed_num_rows, &jacobian_seed_num_cols, // Outputs.
            // Copied from what ADOL-C uses in generate_seed_jac():
            "SMALLEST_LAST", "COLUMN_PARTIAL_DISTANCE_TWO");
    // Convert the seed matrix into an Eigen Matrix for ease of use; delete
    // the memory that ColPack created for the seed matrix.
    const int num_jacobian_seeds = jacobian_seed_num_cols;
    std::cout << "[tropter] Number of finite difference perturbations required "
            "for sparse Jacobian: " << num_jacobian_seeds << std::endl;
    m_jacobian_seed.resize(jacobian_seed_num_rows, jacobian_seed_num_cols);
    for (int i = 0; i < jacobian_seed_num_rows; ++i) {
        for (int j = 0; j < jacobian_seed_num_cols; ++j) {
            m_jacobian_seed(i, j) = jacobian_seed_raw[i][j];
        }
        delete [] jacobian_seed_raw[i];
    }
    delete [] jacobian_seed_raw;


    // Obtain sparsity pattern format to return.
    // -----------------------------------------
    // Provide Jacobian row and column indices in the same order that ColPack
    // will use in jacobian() when recovering the sparse Jacobian from the
    // dense compressed Jacobian.
    // Allocate the recovery object used in jacobian().
    m_jacobian_recovery.reset(new ColPack::JacobianRecovery1D());
    // Allocate memory for the compressed jacobian; we use this in jacobian().
    // Create a lambda that deletes the 2D C array.
    auto double_2d_deleter = [num_jac_rows](double** x) {
        std::for_each(x, x + num_jac_rows, std::default_delete<double[]>());
        delete [] x;
    };
    m_jacobian_compressed = Double2DPtr(new double*[num_jac_rows],
                                        double_2d_deleter);
    for (int i = 0; i < (int)num_jac_rows; ++i) {
        // We don't actually care about the value of m_jacobian_compressed here;
        // no need to set values.
        m_jacobian_compressed[i] = new double[num_jacobian_seeds];
    }
    // Our objective here is to set these vectors from the recovery routine.
    jacobian_row_indices.resize(num_jacobian_nonzeros);
    jacobian_col_indices.resize(num_jacobian_nonzeros);
    unsigned int* jac_row_ptr = jacobian_row_indices.data();
    unsigned int* jac_col_ptr = jacobian_col_indices.data();
    // Again, we don't actually need Jacobian values right now.
    std::vector<double> jacobian_values_dummy(num_jacobian_nonzeros);
    double* jacobian_values_dummy_ptr = jacobian_values_dummy.data();
    m_jacobian_recovery->RecoverD2Cln_CoordinateFormat_usermem(
            m_jacobian_coloring.get(),
            m_jacobian_compressed.get(), m_jacobian_pattern_ADOLC_format.get(),
            &jac_row_ptr, &jac_col_ptr, &jacobian_values_dummy_ptr);

    // Allocate memory that is used in jacobian().
    m_constr_pos.resize(num_jac_rows);
    m_constr_neg.resize(num_jac_rows);
    m_jacobian_compressed_eigen.resize(num_jac_rows, num_jacobian_seeds);
    // Used as dummy variables:
    m_jacobian_recovered_row_indices.resize(num_jacobian_nonzeros);
    m_jacobian_recovered_col_indices.resize(num_jacobian_nonzeros);

    // TODO move ColPack code into a separate file.


    // Hessian.
    // ========
    // Exact hessian mode is unsupported for now.
    if (m_problem.get_use_supplied_sparsity_hessian_lagrangian()) {
        m_problem.calc_sparsity_hessian_lagrangian(x,
                hessian_row_indices, hessian_col_indices);
        if (hessian_row_indices.size() != hessian_col_indices.size()) {
            throw std::runtime_error("Expected hessian_row_indices (size " +
                    std::to_string(hessian_row_indices.size()) + ") and "
                    "hessian_col_indices (size " +
                    std::to_string(hessian_col_indices.size()) +
                    ") to have the same size.");
        }
    } else {
        hessian_row_indices.clear();
        hessian_col_indices.clear();
        // TODO
        //const auto& num_vars = get_num_variables();
        //// Dense upper triangle.
        //unsigned int num_hessian_elements = num_vars * (num_vars - 1) / 2;
        //hessian_row_indices.resize(num_hessian_elements);
        //hessian_col_indices.resize(num_hessian_elements);
    }
}

void OptimizationProblem<double>::Decorator::
calc_objective(unsigned num_variables, const double* variables,
        bool /*new_x*/,
        double& obj_value) const
{
    // TODO avoid copy.
    const VectorXd xvec = Eigen::Map<const VectorXd>(variables, num_variables);
    m_problem.calc_objective(xvec, obj_value);
}

void OptimizationProblem<double>::Decorator::
calc_constraints(unsigned num_variables, const double* variables,
        bool /*new_variables*/,
        unsigned num_constraints, double* constr) const
{
    // TODO avoid copy.
    m_x_working = Eigen::Map<const VectorXd>(variables, num_variables);
    VectorXd constrvec(num_constraints); // TODO avoid copy.
    // TODO at least keep constrvec as working memory.
    m_problem.calc_constraints(m_x_working, constrvec);
    // TODO avoid copy.
    std::copy(constrvec.data(), constrvec.data() + num_constraints, constr);
}

void OptimizationProblem<double>::Decorator::
calc_gradient(unsigned num_variables, const double* x, bool /*new_x*/,
        double* grad) const
{
    m_x_working = Eigen::Map<const VectorXd>(x, num_variables);

    // TODO use a better estimate for this step size.
    const double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    const double two_eps = 2 * eps;

    // We only compute the entries that are nonzero, and we must make sure
    // all other entries are 0.
    std::fill(grad, grad + num_variables, 0);

    double obj_pos;
    double obj_neg;
    // TODO parallelize.
    // "firstprivate" means that each thread will get its own copy of
    // m_x_working, and that it will be copy constructed from m_x_working.
    // All other variables are shared across threads.
    // TODO speedup in Release using OpenMP requires setting environment var
    // OMP_WAIT_POLICY=passive.
    // TODO add `if(parallel)`
    //#pragma omp parallel for
    //            firstprivate(m_x_working)
    //            private(obj_pos, obj_neg)
    for (const auto& i : m_gradient_nonzero_indices) {
        // Perform a central difference.
        m_x_working[i] += eps;
        m_problem.calc_objective(m_x_working, obj_pos);
        m_x_working[i] = x[i] - eps;
        m_problem.calc_objective(m_x_working, obj_neg);
        // Restore the original value.
        m_x_working[i] = x[i];
        grad[i] = (obj_pos - obj_neg) / two_eps;
    }
}

void OptimizationProblem<double>::Decorator::
calc_jacobian(unsigned num_variables, const double* variables, bool /*new_x*/,
        unsigned /*num_nonzeros*/, double* jacobian_values) const
{
    // TODO give error message that sparsity() must be called first.

    // TODO scale by magnitude of x.
    const double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    const double two_eps = 2 * eps;
    // Number of perturbation directions.
    const Eigen::Index num_seeds = m_jacobian_seed.cols();
    Eigen::Map<const VectorXd> x0(variables, num_variables);

    // Compute the dense "compressed Jacobian" using the directions ColPack
    // told us to use.
    // TODO for OpenMP: LowOrder has working memory!
    //#pragma omp parallel for firstprivate(m_constr_pos, m_constr_neg)
    for (Eigen::Index iseed = 0; iseed < num_seeds; ++iseed) {
        const auto direction = m_jacobian_seed.col(iseed);
        // Perturb x in the positive direction.
        m_problem.calc_constraints(x0 + eps * direction, m_constr_pos);
        // Perturb x in the negative direction.
        m_problem.calc_constraints(x0 - eps * direction, m_constr_neg);
        // Compute central difference.
        m_jacobian_compressed_eigen.col(iseed) =
                (m_constr_pos - m_constr_neg) / two_eps;

        // Store this column of the compressed Jacobian in the data structure
        // that ColPack will use.
        for (unsigned int i = 0; i < get_num_constraints(); ++i) {
            m_jacobian_compressed[i][iseed] =
                    m_jacobian_compressed_eigen(i, iseed);
        }
    }

    // Convert the dense compressed Jacobian into the sparse Jacobian layout
    // (specified as triplets {row indices, column indices, values}).
    unsigned int* row_ptr = m_jacobian_recovered_row_indices.data();
    unsigned int* col_ptr = m_jacobian_recovered_col_indices.data();
    m_jacobian_recovery->RecoverD2Cln_CoordinateFormat_usermem(
            m_jacobian_coloring.get(), // ColPack's graph coloring object.
            m_jacobian_compressed.get(), // Holds the finite differences.
            m_jacobian_pattern_ADOLC_format.get(), // Input sparsity pattern.
            &row_ptr, &col_ptr, // Row and col. indices of nonzeros; not needed.
            &jacobian_values); // Corresponding values in the Jacobian.
}

void OptimizationProblem<double>::Decorator::
calc_hessian_lagrangian(unsigned /*num_variables*/, const double* /*variables*/,
        bool /*new_x*/, double /*obj_factor*/,
        unsigned /*num_constraints*/, const double* /*lambda*/,
        bool /*new_lambda TODO */,
        unsigned /*num_nonzeros*/, double* /*hessian_values*/) const
{
    // TODO
    std::string msg =
            "[tropter] Hessian not available with finite differences.";
    std::cerr << msg << std::endl;
    throw std::runtime_error(msg);
}

} // namespace tropter

//#ifdef TROPTER_WITH_OPENMP && _OPENMP
//    #if defined(__GNUC__)
//        #pragma GCC diagnostic pop
//    #elif defined(_MSC_VER)
//        #pragma warning(pop)
//    #endif
//#endif
