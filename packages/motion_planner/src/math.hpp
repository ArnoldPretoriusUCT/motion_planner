// include standard C++ header files
#include <cmath>
#include <vector>

// declare functions
bool equal(const double& a, const double& b);
double modulo(const double& a, const double& b);

// define functions
bool equal(const double& a, const double& b) {
    bool e;
    
    e = std::abs(a - b) < 0.0001;
    
    return e;
}

bool equal_configurations(const std::vector<double>& a, const std::vector<double>& b, const double& M_2PI) {
    bool e;
    
    e = equal(a[0], b[0]) && equal(a[1], b[1]) && (modulo(modulo(a[2], M_2PI) - modulo(b[2], M_2PI), M_2PI) < 0.0001 || modulo(modulo(b[2], M_2PI) - modulo(a[2], M_2PI), M_2PI) < 0.0001);
    
    return e;
}

std::vector<std::vector<double>> flow_matrix_exponential(const std::vector<std::vector<double>>& A) {
    double cosine, division, sine;
    std::vector<std::vector<double>> B;

    if (A[1][0]) {
        sine = std::sin(A[1][0]);
        cosine = std::cos(A[1][0]);
        division = A[0][2] / A[1][0];
        B = {{cosine, -sine, sine * division}, {sine, cosine, (1 - cosine) * division}, {0, 0, 1}};
    }
    else {
        B = {{1, 0, A[0][2]}, {0, 1, 0}, {0, 0, 1}};
    }

    return B;
}

bool greater_or_equal(const double& a, const double& b) {
    bool ge;
    
    ge = a > b || equal(a, b);
    
    return ge;
}

bool less_or_equal(const double& a, const double& b) {
    bool le;
    
    le = a < b || equal(a, b);
    
    return le;
}

std::vector<double> linear_space(const double& a, const double& b, const int& number_of_steps) {
    int i;
    double step;
    std::vector<double> s;
    
    if (number_of_steps) {
        if (number_of_steps > 2) {
            s = std::vector<double>(number_of_steps - 1, 0);
            step = (b - a) / (number_of_steps - 1);
            for (i = 0; i < number_of_steps - 1; i++) {
                s[i] = a + step * (i + 1);
            }
            s.back() = b;
        }
        else {
            s = {b};
        }
    }
    
    return s;
}

std::vector<std::vector<double>> matrix_addition(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    int i, j, number_of_columns, number_of_rows;
    std::vector<std::vector<double>> C;
    
    number_of_rows = A.size();
    number_of_columns = A[0].size();
    C = std::vector<std::vector<double>>(number_of_rows, std::vector<double>(number_of_columns, 0));
    for (i = 0; i < number_of_rows; i++) {
        for (j = 0; j < number_of_columns; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }

    return C;
}

std::vector<std::vector<double>> matrix_multiplication(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    int A_rows, B_columns, i, j, k;
    std::vector<std::vector<double>> C;

    A_rows = A.size();
    B_columns = B[0].size();
    C = std::vector<std::vector<double>>(A_rows, std::vector<double>(B_columns, 0));
    for (i = 0; i < A_rows; i++) {
        for (j = 0; j < B_columns; j++) {
            for (k = 0; k < A[0].size(); k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    return C;
}

std::vector<std::vector<double>> matrix_subtraction(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    int i, j, number_of_columns, number_of_rows;
    std::vector<std::vector<double>> C;
    
    number_of_rows = A.size();
    number_of_columns = A[0].size();
    C = std::vector<std::vector<double>>(number_of_rows, std::vector<double>(number_of_columns, 0));
    for (i = 0; i < number_of_rows; i++) {
        for (j = 0; j < number_of_columns; j++) {
            C[i][j] = A[i][j] - B[i][j];
        }
    }

    return C;
}

std::vector<std::vector<double>> matrix_transposition(const std::vector<std::vector<double>>& A) {
    int i, j, number_of_columns, number_of_rows;
    std::vector<std::vector<double>> B;

    number_of_rows = A.size();
    number_of_columns = A[0].size();
    B = std::vector<std::vector<double>>(number_of_columns, std::vector<double>(number_of_rows, 0));
    for (i = 0; i < number_of_columns; i++) {
        for (j = 0; j < number_of_rows; j++) {
            B[i][j] = A[j][i];
        }
    }

    return B;
}

double modulo(const double& a, const double& b) {
    double m;
    
    if (equal(a, 0)) {
        m = 0;
    }
    else {
        m = a - b * std::floor(a / b);
    }

    return m;
}

std::vector<std::vector<double>> scale_matrix(const std::vector<std::vector<double>>& A, const double& a) {
    int i, j, number_of_columns, number_of_rows;
    std::vector<std::vector<double>> B;
    
    number_of_rows = A.size();
    number_of_columns = A[0].size();
    B = std::vector<std::vector<double>>(number_of_rows, std::vector<double>(number_of_columns, 0));
    for (i = 0; i < number_of_rows; i++) {
        for (j = 0; j < number_of_columns; j++) {
            B[i][j] = a * A[i][j];
        }
    }

    return B;
}

std::vector<std::vector<double>> two_by_two_matrix_inversion(const std::vector<std::vector<double>>& A) {
    double determinant;
    std::vector<std::vector<double>> B;

    determinant = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    B = {{A[1][1] / determinant, -A[0][1] / determinant}, {-A[1][0] / determinant, A[0][0] / determinant}};

    return B;
}
