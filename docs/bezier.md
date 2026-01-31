# Bezier interpolation

Bezier interpolation is a recursive linear interpolation with $n$ points.
Between all consecutive pairs of points a linear interpolation is computed with the same parameter.
After applying all linear interpolations once there are $n-1$ intermediate points.
With every iteration of linear interpolations the number of intermediate points is reduced by one.
This procedure is continued until only one point is left which is the result of the bezier interpolation.

This recursive defintion of the bezier interpolation is called the [de casteljau algorithm](https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm).

## Example for 4 points

Given the 4 ordered points $\vec p_0$, $\vec p_1$, $\vec p_2$ and $\vec p_3$ and the interpolation parameter $k$ the first iteration yields the 3 points $\vec q_0$, $\vec q_1$ and $\vec q_2$  as

$$ \vec q_0 = \vec p_0 (1-k) + \vec p_1 k $$
$$ \vec q_1 = \vec p_1 (1-k) + \vec p_2 k $$
$$ \vec q_2 = \vec p_2 (1-k) + \vec p_3 k $$

The second iteration yields $\vec r_0$ and $\vec r_1$ as

$$ \vec r_0 = \vec q_0 (1-k) + \vec q_1 k $$
$$ \vec r_1 = \vec q_1 (1-k) + \vec q_2 k $$

Finally the third iteration yields the interpolation result $\vec s$

$$ \vec s = \vec r_0 (1-k) + \vec r_1 t $$

## Derivatives via de casteljau

The point $\vec p_{i,j}$ where $i$ is the iteration and $j$ is the index is computed as

$$ \vec p_{i,j} = \vec p_{i-1, j} (1-k) + \vec p_{i-1, j+1} k $$

### First derivative

The first derivative with respect to $k$ is defined as

$$ \dot{\vec p}_{i,j} = \dot{\vec p}_{i-1, j} (1-k) - \vec p_{i-1, j} + \dot{\vec p}_{i-1, j+1} k + \vec p_{i-1, j+1} $$

Note that the original points defining the bezier interpolation are constant with respect to $k$. So $\dot{\vec p}_{0,j} = 0 $.

### Second derivative via de castelja

Similarly the second derivative is defined as

$$ \ddot{\vec p}_{i,j} = \ddot{\vec p}_{i-1, j} (1-k) - 2 \dot{\vec p}_{i-1, j} + \ddot{\vec p}_{i-1, j+1} k + 2 \dot{\vec p}_{i-1, j+1} $$

Note that the original points defining the bezier interpolation are constant with respect to $k$. So $\ddot{\vec p}_{0,j} = 0 $.
