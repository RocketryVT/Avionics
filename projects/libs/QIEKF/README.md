# QIEKF

This is an implementation of the paper:

- "Quaternion Invariant Extended Kalman Filtering for Spacecraft Attitude Estimation"
- <https://arc.aiaa.org/doi/10.2514/1.G003177>
- Specifically only the RIEKF variant is implemented here.

## Paper Summary

### Nomenclature

| Symbol | Description |
| :--- | :--- |
| $A(q)$ | 3-by-3 rotation matrix as a function of the attitude quaternion $q$ |
| $I_n$ | $n \times n$ identity matrix |
| $P$ | Covariance matrix of the state estimation error |
| $q$ | The spacecraft attitude quaternion |
| $r_i$ | The $i$-th known vector expressed in the inertial frame |
| $F_B$ | Spacecraft body-fixed frame |
| $F_I$ | Inertial frame |
| $H$ | The set of quaternions |
| $S^3$ | The set of unit quaternions |
| $\omega$ | Spacecraft angular velocity vector expressed in $F_B$, rad/s |
| $\bar{\omega}$ | Measurement of the spacecraft angular velocity vector, rad/s |
| $\beta$ | Gyro bias, rad/s |
| $\eta_v, \eta_u$ | Gyro noise processes |
| $v_i$ | White Gaussian noise process associated with the $i$-th vector measurement |
| $g$ | Group element and serves as the state variable $x$ with $g = (q, \beta)$ |
| $u$ | The system input variable defined as $u = (\bar{\omega}, r_1, \dots, r_n)$ |
| $\|\|\cdot\|\|$ | Euclidean norm of a vector |

### Superscripts

| Symbol | Description |
| :--- | :--- |
| $(\cdot)^T$ | Transpose of a vector or matrix |
| $\hat{(\cdot)}$ | Estimate of a state variable |
| $\tilde{(\cdot)}$ | Estimation error of a state variable |
| $(\cdot)^-$ | A priori estimate of a variable |
| $(\cdot)^+$ | A posteriori estimate of a variable |

### Subscripts

| Symbol | Description |
| :--- | :--- |
| $(\cdot)_k$ | The value of a vector or matrix at time $t_k$ |

## RIEKF Formulation (pg. 16 of the paper)

### Initialize

$$
\begin{aligned}
\hat{q}_0 &= \hat{q}(t_0) \\
\hat{\beta}_0 &= \hat{\beta}(t_0) \\
P_0 &= P(t_0)
\end{aligned}
$$

### Propagation

$$
\begin{cases}
\dot{\hat{q}} = \frac{1}{2} \hat{q} \otimes (\bar{\omega} - \hat{\beta}) \\
\dot{\hat{\beta}} = 0 \\
\dot{P} = FP + PF^T + GQG^T
\end{cases}
$$

### Update

$$
\begin{cases}
K_k = P_k^- H_k^T (H_k P_k^- H_k^T + \hat{R}_k)^{-1} \\
P_k^+ = (I_6 - K_k H_k) P_k^- \\
\begin{bmatrix} c_{q,k} \\ c_{\beta,k} \end{bmatrix} = K_k E(\hat{g}_k^-, u_k, y_k) \\
\hat{q}_k^+ = \exp_q \left( -\frac{c_{q,k}}{2} \right) \otimes \hat{q}_k^- \\
\hat{\beta}_k^+ = \hat{\beta}_k^- - (\hat{q}_k^+)^* \otimes c_{\beta,k} \otimes \hat{q}_k^+ \\
\hat{\omega}_k^+ = \bar{\omega}_k - \hat{\beta}_k^+
\end{cases}
$$

**Note:** The matrices $F$, $G$, and $Q$ are given in Eq. (27); $E(\hat{g}_k^-, u_k, y_k)$, $H_k$, and $\hat{R}_k$ are given in Eqs. (29), (31) and (34) respectively.

Equations (24) and (25) can be formulated in the matrix form:

$$
\begin{bmatrix} \dot{\tilde{\gamma}} \\ \dot{\tilde{\beta}} \end{bmatrix} = F \begin{bmatrix} \tilde{\gamma} \\ \tilde{\beta} \end{bmatrix} + G \begin{bmatrix} \eta_v \\ \eta_u \end{bmatrix} \tag{26}
$$

where

$$
F = \begin{bmatrix} 0_{3 \times 3} & -I_3 \\ 0_{3 \times 3} & \hat{I} \times \omega \end{bmatrix}, \quad G = \begin{bmatrix} A^T(\hat{q}) & 0_{3 \times 3} \\ 0_{3 \times 3} & -A^T(\hat{q}) \end{bmatrix}
$$

$$
Q = E \left\{ \begin{bmatrix} \eta_v \\ \eta_u \end{bmatrix} \begin{bmatrix} \eta_v^T & \eta_u^T \end{bmatrix} \right\} = \begin{bmatrix} \sigma_v^2 I_3 & 0_{3 \times 3} \\ 0_{3 \times 3} & \sigma_u^2 I_3 \end{bmatrix}
$$

Equations (29), (31), and (34) are given by:

$$
E(\hat{g}_k^-, u_k, y_k) = \begin{bmatrix} r_{1k} \\ \vdots \\ r_{nk} \end{bmatrix} - \begin{bmatrix} \hat{q}_k^- \otimes y_{1k} \otimes (\hat{q}_k^-)^* \\ \vdots \\ \hat{q}_k^- \otimes y_{nk} \otimes (\hat{q}_k^-)^* \end{bmatrix} \tag{29}
$$

Consequently, the linearization of Eq. (29) is derived as

$$
E(\hat{g}_k^-, u_k, y_k) = H_k \begin{bmatrix} \tilde{\gamma}_k^- \\ \tilde{\beta}_k^- \end{bmatrix} + \hat{V}_k
$$

where

$$
H_k = \begin{bmatrix} (r_{1k}) \times & 0_{3 \times 3} \\ \vdots & \vdots \\ (r_{nk}) \times & 0_{3 \times 3} \end{bmatrix}, \quad \hat{V}_k = \begin{bmatrix} A^T(\hat{q}_k^-) & 0_{3 \times 3} \\ \vdots & \vdots \\ 0_{3 \times 3} & A^T(\hat{q}_k^-) \end{bmatrix} V_k \tag{31}
$$

$$
R_k = E \{ \hat{V}_k \hat{V}_k^T \} = \begin{bmatrix} A^T(\hat{q}_k^-) & 0_{3 \times 3} \\ \vdots & \vdots \\ 0_{3 \times 3} & A^T(\hat{q}_k^-) \end{bmatrix} R_k \begin{bmatrix} A(\hat{q}_k^-) & 0_{3 \times 3} \\ \vdots & \vdots \\ 0_{3 \times 3} & A(\hat{q}_k^-) \end{bmatrix} \tag{34}
$$