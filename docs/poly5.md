# Polynomial of order 5

The function and its first and second derivative are the following: 

$$ \vec f(x) = \vec a_0 + \vec a_1 x + \vec a_2 x^2 + \vec a_3 x^3 + \vec a_4 x^4 + \vec a_5 x^5 $$
$$ \vec f'(x) = \vec a_1 + 2 \vec a_2 x + 3 \vec a_3 x^2 + 4 \vec a_4 x^3 + 5 \vec a_5 x^4 $$
$$ \vec f''(x) = 2 \vec a_2 + 6 \vec a_3 x + 12 \vec a_4 x^2 + 20 \vec a_5 x^3 $$

When setting x to $ 0 $ or $ 1 $ as these are the limits of interpolation, the following expressions for these constraints are

$$ \vec f(0) = \vec a_0 $$
$$ \vec f'(0) = \vec a_1 $$
$$ \vec f''(0) = 2 \vec a_2 $$
$$ \vec f(1) = \vec a_0 + \vec a_1 + \vec a_2 + \vec a_3 + \vec a_4 + \vec a_5 $$
$$ \vec f'(1) = \vec a_1 + 2 \vec a_2 + 3 \vec a_3 + 4 \vec a_4 + 5 \vec a_5 $$
$$ \vec f''(1) = 2 \vec a_2 + 6 \vec a_3 + 12 \vec a_4 + 20 \vec a_5 $$

The first three coefficients can be computed directly by

$$ \vec a_0 = \vec f(0) $$
$$ \vec a_1 = \vec f'(0) $$
$$ \vec a_2 = \frac{\vec f''(0)}{2} $$

Since all dimensions are completely independent, for every component there is a set of three linear equalations

$$ f_i(1) - f_i(0) - f'_i(0) - \frac{f''_i(0)}{2} = c_i = a_{3,i} + a_{4,i} + a_{5,i} $$
$$ f'_i(1) - f'_i(0) - f''_i(0) = d_i = 3 a_{3,i} + 4 a_{4,i} + 5 a_{5,i} $$
$$ f''_i(1) - f''_i(0) = e_i = 6 a_{3,i} + 12 a_{4,i} + 20 a_{5,i} $$

Which can be transformed into matrix vector form

$$ \boxed R \cdot \vec a = \vec b $$

with

$$ \boxed R = \begin{bmatrix} 1 & 1 & 1\\3 & 4 & 5\\6 & 12 & 20 \end{bmatrix} $$
$$ \vec a = \begin{bmatrix} a_{3,i}\\a_{4,i}\\a_{5,i} \end{bmatrix} $$
$$ \vec b = \begin{bmatrix} c_i\\d_i\\e_i \end{bmatrix} $$

By inversing the matrix $ \boxed R $ the last three coefficients can be computed

$$ \vec a = \boxed R ^ {-1} \cdot \vec b $$
$$ \boxed R ^ {-1} = 
\begin{bmatrix}
10 & -4 & \frac12\\
-15 & 7 & -1\\
6 & -3 & \frac12
\end{bmatrix}
$$

$$ a_{3,i} = 10 c_i - 4 d_i + \frac{e_i}{2} $$
$$ a_{4,i} = -15 c_i + 7 d_i - e_i $$
$$ a_{5,i} = 6 c_i - 3 d_i + \frac{e_i}{2} $$

Solving this for all dimensions yields

$$ \vec a_3 = 10 \vec c - 4 \vec d + \frac12 \vec e $$
$$ \vec a_4 = -15 \vec c + 7 \vec d - \vec e $$
$$ \vec a_5 = 6 \vec c - 3 \vec d + \frac12 \vec e $$

