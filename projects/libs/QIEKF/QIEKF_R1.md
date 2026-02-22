# Quaternion Invariant Extended Kalman Filtering for Spacecraft Attitude Estimation

(Note this was converted from PDF to Markdown using an automated tool, needs to be checked against the original paper for accuracy.)

**Authors:** Haichao Gui (Beihang University) and Anton H. J. de Ruiter (Ryerson University)

## Abstract

The spacecraft attitude estimation is addressed in the framework of invariant Kalman filtering, which rests on invariance of the system dynamics and output map with respect to appropriate coordinate transformations. The available measurements are assumed to be the angular velocity from three-axis gyroscopes and vector measurements from attitude sensors. Two continuous-discrete quaternion filters are developed from output-state errors defined in the inertial frame and the spacecraft body frame respectively. The former is termed the right-invariant extended Kalman filter (RIEKF) and the latter left-invariant extended Kalman filter (LIEKF). These two filters both respect the norm constraint of the attitude quaternion but stem from different invariance properties of the system dynamics. It is shown that the LIEKF bears much resemblance to, and thus can be viewed as a minor variant of the conventional quaternion multiplicative extended Kalman filter (MEKF). The RIEKF possesses less dependence on the estimated trajectory and, as a result, better robustness than the LIEKF and MEKF. Extensive Monte Carlo simulations of spacecraft attitude determination implementations demonstrate the advantageous performance of the RIEKF, compared to the LIEKF, MEKF, and some of their improved variants.

---

## Nomenclature

* **$A(q)$**: 3-by-3 rotation matrix as a function of the attitude quaternion $q$
* **$I_{n}$**: $n \times n$ identity matrix
* **$P$**: Covariance matrix of the state estimation error
* **$q$**: The spacecraft attitude quaternion
* **$r^i$**: The $i$-th known vector expressed in the inertial frame
* **$\mathcal{F}_{B}$**: Spacecraft body-fixed frame
* **$\mathcal{F}_{I}$**: Inertial frame
* **$\mathbb{H}$**: The set of quaternions
* **$\mathbb{S}^{3}$**: The set of unit quaternions
* **$\omega$**: Spacecraft angular velocity vector expressed in $\mathcal{F}_{B}$, rad/s
* **$\overline{\omega}$**: Measurement of the spacecraft angular velocity vector, rad/s
* **$\beta$**: Gyro bias, rad/s
* **$\eta_{v}, \eta_{u}$**: Gyro noise processes
* **$v^{i}$**: White Gaussian noise process associated with the $i$-th vector measurement
* **$g$**: Group element serving as the state variable with $g=(q,\beta)$
* **$u$**: The system input variable defined as $u=(\overline{\omega},r^{1},\dots,r^{n})$
* **$||\cdot||$**: Euclidean norm of a vector

**Superscripts:**

* $T$: Transpose of a vector or matrix
* $\hat{x}$: Estimate of a state variable
* $\tilde{x}$: Estimation error of a state variable
* $-$: A priori estimate of a variable
* $+$: A posteriori estimate of a variable

**Subscripts:**

* $k$: The value of a vector or matrix at time $t_{k}$

---

## I. Introduction

The knowledge of the spacecraft attitude, which forms a 3-dimensional compact manifold, is essential for its on-orbit operation. Since the spacecraft attitude cannot be directly measured, it is usually derived from body-frame observations of reference vectors, usually complemented with angular velocity measurements from gyroscopes for improved accuracy. Various attitude estimation algorithms have been proposed, including algebraic approaches using merely vector measurements such as TRIAD, QUEST, and ESOQ, and dynamical approaches that combine sensor measurements with spacecraft attitude equations such as extended Kalman filters (EKF) and deterministic observers. A comprehensive survey of spacecraft attitude estimation algorithms can be found in [13].

Among the existing methods, the multiplicative EKF (MEKF) in terms of quaternions has achieved broad applications because it is nonsingular and involves fewer parameters than the SO(3) formulation. The fundamental idea is to estimate the attitude error with three-parameter formulations while using a unit quaternion to represent the estimated attitude. This idea does not entail, but some implementations of the MEKF still require, a brute-force normalization step in the filtering algorithm. In order to accommodate this issue, continuous-time and discrete-time constrained Kalman filters (CKF) preserving the norm constraint were developed. It was shown that brute-force normalization after the measurement update, as adopted in the quaternion MEKF, is in fact optimal.

The EKF methods work by linearizing nonlinear systems about the estimated trajectory and then applying the Kalman filter to the linearized system. Hence, their performance largely relies on validity of the small error assumption and is sensitive to the initial state guess. With poor initial estimation, the EKF can exhibit degraded performance or even diverge. To overcome this drawback, improvements within the EKF framework were presented, such as an extended QUEST algorithm and a q-method EKF. Reynolds introduced a covariance resetting after measurement update with proven convergence for attitude estimation without gyroscope bias.

Recently, the invariant extended Kalman filtering (IEKF) has emerged as an appealing improvement to the conventional EKF with comparable computational efficiency. It applies to dynamical systems possessing invariance properties. Here, invariance refers to the case that the system dynamics and the corresponding output map stay unchanged with respect to proper coordinate transformations on the state, input, and output variables. The IEKF performs the measurement update in a nonlinear manner that preserves the geometry of the state manifold. When the invariant output error and state estimation error are appropriately constructed, the dependence of the resultant IEKF on the estimated trajectory can be reduced or even eliminated, yielding increased robustness.

This paper studies the spacecraft attitude determination issue using angular velocity measurements from rate gyros and vector measurements from attitude sensors. Two quaternion IEKFs involving continuous dynamics and discrete measurements are developed: the right-invariant extended Kalman filter (RIEKF) and the left-invariant extended Kalman filter (LIEKF). Both filters preserve the unit-norm constraint of attitude quaternions by generating corrections with the quaternion exponential map but their individual structures differ greatly. The RIEKF features an invariant output error and a state estimation error evaluated in the inertial frame. In contrast, the LIEKF features an invariant output error and a state estimation error evaluated in the spacecraft body frame, which are exactly the same as those of the classical quaternion MEKF.

---

## II. Preliminaries and System Models

### A. Quaternions

Denote by $\mathbb{H}$ the set of quaternions, which is actually isomorphic to $\mathbb{R}^{4}$. A quaternion can be represented as $q=[q_{0},q_{v}^{T}]^{T}\in\mathbb{H}$, where $q_{0}\in\mathbb{R}$ and $q_{v}=[q_{1},q_{2},q_{3}]^{T}\in\mathbb{R}^{3}$ are called the scalar and vector parts of $q$, respectively. The conjugate of $q$ is given by $q^{*}=[q_{0},-q_{v}^{T}]^{T}$. The quaternion product between $q$ and any $p=[p_{0},p_{v}^{T}]^{T}\in\mathbb{H}$ is defined as:

$$q\otimes p=\begin{bmatrix}q_{0}p_{0}-q_{v}^{T}p_{v}\\ q_{0}p_{v}+p_{0}q_{v}+q_{v}\times p_{v}\end{bmatrix}$$

Unit quaternions are quaternions with unit length and comprise the unit sphere in $\mathbb{R}^{4}$, denoted by $\mathbb{S}^{3}=\{q\in\mathbb{H}:q\otimes q^{*}=q^{*}\otimes q=1\}$. Given a rotation about the unit axis $\eta\in\mathbb{R}^{3}$ at an angle $\phi\in\mathbb{R}$, the corresponding unit quaternion is:

$$q=\begin{bmatrix}\cos(\frac{\phi}{2})\\ \eta\sin(\frac{\phi}{2})\end{bmatrix} \tag{1}$$

This gives rise to a rotation matrix $A(q)$:

$$A(q) = (q_0^2 - ||q_v||^2)I_3 + 2q_v q_v^T + 2q_0 q_v^\times \tag{2}$$

*(Note: Reconstructed from standard Rodrigues formula implied by text symbols)*. For all $x\in\mathbb{R}^{3}$, $q$ and $A(q)$ satisfy:

$$A(q)x=q^{*}\otimes x\otimes q$$
$$A^{T}(q)x=A(q^{*})x=q\otimes x\otimes q^{*}$$

The quaternion exponential map from $\mathbb{R}^{3}$ to $\mathbb{S}^{3}$ is given by:

$$\exp_{q}(x)=\sum_{n=0}^{\infty}\frac{1}{n!}\frac{(x\otimes\cdots\otimes x)}{n} = \begin{bmatrix}\cos(||x||)\\ \frac{x}{||x||}\sin(||x||)\end{bmatrix} \tag{3, 4}$$

### B. System Model

Let $q(t)\in\mathbb{S}^{2}$ represent the attitude of the spacecraft relative to the inertial frame $\mathcal{F}_{I}$. The attitude kinematics are:

$$\dot{q}(t)=\frac{1}{2}q(t)\otimes\omega(t) \tag{5}$$

The spacecraft angular velocity measurement model is:

$$\omega(t)=\overline{\omega}(t)-\beta(t)-\eta_{v}(t) \tag{6}$$
$$\dot{\beta}(t)=\eta_{u}(t) \tag{7}$$

Vector measurements are modeled as:

$$y^{i}(t)=q^{*}(t)\otimes r^{i}(t)\otimes q(t)+v^{i}(t), \quad i=1,\dots,n$$

These can be written in aggregated form:

$$y=h(q,\beta,u)+V, \quad h(q,\beta,u)=\begin{bmatrix}q^{*}\otimes r^{1}\otimes q\\ \vdots\\ q^{*}\otimes r^{n}\otimes q\end{bmatrix} \tag{8}$$

### C. Invariance Properties

Consider the smooth system:

$$\begin{cases}\dot{q}=\frac{1}{2}q\otimes(\overline{\omega}-\beta)\\ \dot{\beta}=0\\ y=h(q,\beta,u)\end{cases} \tag{9}$$

Definition 1: The system is G-invariant if $d\varphi_{g}(x)/dt=f(\varphi_{g}(x),\psi_{g}(u))$ for all $g, x, u$.
Definition 2: The output is G-equivariant if $h(\varphi_{g}(x), \psi_{g}(u))=\rho_{g}(h(x,u))$.

---

## III. Right Invariant Extended Kalman Filter Design

### A. Invariant Output Errors and Estimation Errors

$\mathbb{S}^{3}\times\mathbb{R}^{3}$ forms a Lie group under the following group composition:

$$\begin{pmatrix}q_{r}\\ \beta_{r}\end{pmatrix}\circ\begin{pmatrix}q\\ \beta\end{pmatrix}=\begin{pmatrix}q\otimes q_{r}\\ q_{r}^{*}\otimes\beta\otimes q_{r}+\beta_{r}\end{pmatrix} \tag{11}$$

Let $G=\mathbb{S}^{3}\times\mathbb{R}^{3}$. The inverse of $g$ is:

$$g^{-1}=\begin{pmatrix}q^{*}\\ -q\otimes\beta\otimes q^{*}\end{pmatrix} \tag{12}$$

The group actions are defined as $\varphi_{g_{r}}(g)=g_{r}\circ g$ and $\psi_{g_{r}}(u)=(q_{r}^{*}\otimes(\overline{\omega}+\beta_{r})\otimes q_{r},r^{1},\dots,r^{n})$. The invariant output error is derived as:

$$E(\hat{g},u, y) = \rho_{\hat{g}^{-1}}(h(\hat{g},u)) - \rho_{\hat{g}^{-1}}(y) = \begin{bmatrix} r^1 \\ \vdots \\ r^n \end{bmatrix} - \begin{bmatrix} \hat{q}\otimes y^1 \otimes \hat{q}^* \\ \vdots \\ \hat{q}\otimes y^n \otimes \hat{q}^* \end{bmatrix} \tag{13}$$

The estimation errors are defined as:

$$\tilde{g}(g, \hat{g}) = \begin{pmatrix}\tilde{q}\\ \tilde{\beta}\end{pmatrix} = g^{-1}\circ \hat{g}=\begin{pmatrix}\hat{q}\otimes q^*\\ q \otimes (\hat{\beta}- \beta) \otimes q^*\end{pmatrix} \tag{14}$$

The output error $E(\hat{g},u, y)$ and the bias estimation error $\tilde{\beta}$ are both evaluated in the **inertial frame**.

### B. Propagation and Updating Equations

The RIEKF is proposed with the following recursive steps:

**Propagation:**
$$\dot{\hat{g}} = \begin{pmatrix}\dot{\hat{q}}\\ \dot{\hat{\beta}}\end{pmatrix} = \begin{pmatrix}\frac{1}{2}\hat{q}\otimes (\overline{\omega}- \hat{\beta})\\ 0\end{pmatrix}, \quad t_{k-1} \leq t < t_k \tag{15}$$

**Update:**
$$\hat{g}^+_k = \hat{g}^-_k \circ [\exp_g(K_k E(\hat{g}^-_k,u_k, y_k))]^{-1} \tag{16}$$

where $K_k \in \mathbb{R}^{6\times3n}$ is the filter gain matrix. The update equation can be written as:

$$\hat{g}^+_k =\begin{pmatrix}\hat{q}^+_k\\ \hat{\beta}^+_k\end{pmatrix} =\begin{pmatrix}\exp_q(-\frac{cq_k}{2}) \otimes \hat{q}^-_k\\ \hat{\beta}^-_k - (\hat{q}^+_k)^* \otimes c\beta_k \otimes \hat{q}^+_k\end{pmatrix} \tag{20}$$

where $[cq_k^T, c\beta_k^T]^T = K_k E(\hat{g}^-_k,u_k, y_k)$.

### C. Filtering Gain Tuning

The linearized error dynamics are:

$$\begin{bmatrix}\dot{\tilde{\gamma}}\\ \dot{\tilde{\beta}}\end{bmatrix}= F\begin{bmatrix}\tilde{\gamma}\\ \tilde{\beta}\end{bmatrix}+ G\begin{bmatrix}\eta_v\\ \eta_u\end{bmatrix} \tag{26}$$

where:
$$F = \begin{bmatrix} 0_{3\times3} & -I_3 \\ 0_{3\times3} & \hat{I}_{\omega}^\times \end{bmatrix}, \quad G =\begin{bmatrix}A^T(\hat{q}) & 0_{3\times3}\\ 0_{3\times3} & -A^T(\hat{q})\end{bmatrix} \tag{27}$$
and $\hat{I}_{\omega} = A^T(\hat{q})(\overline{\omega}- \hat{\beta})$.

The invariant output error linearization is:
$$E(\hat{g}^-_k,u_k, y_k) = H_k \begin{bmatrix}\tilde{\gamma}^-_k\\ \tilde{\beta}^-_k\end{bmatrix} + \hat{V}_k \tag{30}$$

where:
$$H_k = \begin{bmatrix}(r^{1}_k)^\times & 0_{3\times3}\\ \vdots & \vdots \\ (r^{n}_k)^\times & 0_{3\times3}\end{bmatrix} \tag{31}$$

The Kalman gain $K_k$ is computed using standard Kalman filter equations involving $P$, $H_k$, and a modified noise covariance $\hat{R}_k$.

---

## IV. Left Invariant Extended Kalman Filter Design

$G = \mathbb{S}^3 \times \mathbb{R}^3$ remains a Lie group with the alternative binary composition:

$$g_r \circ g = \begin{pmatrix}q_r\\ \beta_r\end{pmatrix} \circ\begin{pmatrix}q\\ \beta\end{pmatrix} =\begin{pmatrix}q_r \otimes q\\ \beta_r + \beta\end{pmatrix} \tag{35}$$

This induces the invariant output error and invariant estimation error:

$$E(\hat{g},u, y) = y - h(\hat{g},u) = y - \begin{bmatrix}\hat{q}^* \otimes r^1 \otimes \hat{q}\\ \vdots \\ \hat{q}^* \otimes r^n \otimes \hat{q}\end{bmatrix} \tag{36}$$

$$\tilde{g}(g, \hat{g}) = \hat{g}^{-1}\circ g =\begin{pmatrix}\hat{q}^* \otimes q\\ \beta - \hat{\beta}\end{pmatrix} \tag{37}$$

These errors are evaluated in the **spacecraft body frame**. The update step is:

$$\hat{q}^+_k = \hat{q}^-_k \otimes \exp_q(\frac{cq_k}{2})$$
$$\hat{\beta}^+_k = \hat{\beta}^-_k + c\beta_k$$

The matrices for the LIEKF are:

$$F = \begin{bmatrix} -(\overline{\omega}- \hat{\beta})^\times & -I_3 \\ 0_{3\times3} & 0_{3\times3} \end{bmatrix}, \quad G = \begin{bmatrix} -I_3 & 0_{3\times3} \\ 0_{3\times3} & I_3 \end{bmatrix}$$

$$H_k = \begin{bmatrix}(A(\hat{q}^-_k)r^{1}_k)^\times & 0_{3\times3}\\ \vdots & \vdots \\ (A(\hat{q}^-_k)r^{n}_k)^\times & 0_{3\times3}\end{bmatrix}$$

---

## V. Discussion

* **Relationship to MEKF:** The quaternion LIEKF is mostly identical to the conventional quaternion MEKF except for the attitude correction step. The MEKF utilizes a first-order approximation for correction and requires brute-force normalization, whereas the LIEKF respects the unit-norm constraint naturally. The MEKF can be viewed as a minor variant of the quaternion LIEKF.
* **Robustness:** The RIEKF has less dependence on the estimated trajectory and input. Its sensitivity matrix $H_k$ does not depend on the estimated attitude, unlike the LIEKF.
* **Physical Insight:** The attitude error in the LIEKF (and MEKF) corresponds to the attitude error in the spacecraft body frame. The attitude error in the RIEKF corresponds to the attitude estimation error in the **inertial frame**.

---

## VI. Numerical Examples

Three sets of Monte Carlo simulations were conducted.

### Table 1: Monte Carlo simulation parameters

| Parameter | Section VI.A | Section VI.B | Section VI.C |
| :--- | :--- | :--- | :--- |
| Covariance of Sun sensor noise $v^1$, rad | $0.0017^2 I_3$ | $0.0175^2 I_3$ | $0.0175^2 I_3$ |
| Covariance of magnetometer noise $v^2$, rad | $0.0087^2 I_3$ | $0.0873^2 I_3$ | $0.0873^2 I_3$ |
| Std dev of bias noise $\sigma_v$, rad/$s^{3/2}$ | $\sqrt{10} \times 10^{-7}$ | $\sqrt{10} \times 10^{-7}$ | $\sqrt{10} \times 10^{-5}$ |
| Std dev of gyro noise $\sigma_u$, rad/$s^{3/2}$ | $\sqrt{10} \times 10^{-10}$ | $\sqrt{10} \times 10^{-10}$ | $\sqrt{10} \times 10^{-8}$ |
| Initial attitude estimate $\hat{q}(0)$ | 1 | 1 | 1 |
| Initial bias estimate $\hat{\beta}(0)$, rad/s | $[0, 0, 0]^T$ | $[0, 0, 0]^T$ | $[0, 0, 0]^T$ |
| Initial covariance for attitude error, deg$^2$ | $10^2 I_3$ | $150^2 I_3$ | $10^2 I_3$ |
| Initial covariance for bias error, (deg/h)$^2$ | $3^2 I_3$ | $20^2 I_3$ | $5^2 I_3$ |

**A. Small Initial Estimation Errors:** All filters (MEKF, CKF, LIEKF, RIEKF, CRMEKF, CRLIEKF) provided nearly indistinguishable performances.

**B. Large Initial Estimation Errors:** The RIEKF demonstrated significantly better performance than the other five filters, achieving faster convergence and higher estimation accuracy. The RIEKF takes 10 min to decrease attitude RMSE below 2 deg, whereas others never achieved comparable accuracy.

**C. Severe Initial Condition:** With large initial errors and erroneous small initial covariance, the RIEKF attained better performance in terms of convergence speed and prediction of estimation uncertainty.

---

## VII. Conclusion

Two continuous-discrete quaternion invariant extended Kalman filters (IEKFs) were developed: the RIEKF and LIEKF. The LIEKF is almost identical to the classical MEKF. The RIEKF depends less on the estimated trajectory and achieves better robustness. Simulation results showed that while performance is similar for small errors, the RIEKF demonstrates faster convergence and more accurate estimation in severe scenarios involving large initial errors or erroneous covariances.

---

## References

1. Black, H. D., "A Passive System for Determining the Attitude of a Satellite," AIAA Journal, Vol. 2, No. 7, 1964.

2. Bar-Itzhack, I. Y. and Reiner, J., "Recursive Attitude Determination from Vector Observations: Direction Cosine Matrix Identification," AIAA Journal of Guidance, Control, and Dynamics, Vol. 7, No. 1, 1984.

3. Shuster, M. D. and Oh, S. D., "Three-Axis Attitude Determination from Vector Observations," AIAA Journal of Guidance, Control, and Dynamics, Vol. 4, No. 1, 1981.

4. Bar-Itzhack, I. Y., "REQUEST: A Recursive QUEST Algorithm for Sequential Attitude Determination," AIAA Journal of Guidance, Control, and Dynamics, Vol. 19, No. 5, 1996.

5. Mortari, D., "ESOQ: A Closed-Form Solution to the Wahba Problem," Journal of the Astronautical Sciences, Vol. 45, No. 2, 1997.

6. Lefferts, E. J., Markley, F. L., and Shuster, M. D., "Kalman Filtering for Spacecraft Attitude Estimation," AIAA Journal of Guidance, Control, and Dynamics, Vol. 5, No. 5, 1982.

7. Psiaki, M. L. and Oshman, Y., "Three-Axis Attitude Determination via Kalman Filtering of Magnetometer Data," AIAA Journal of Guidance, Control, and Dynamics, Vol. 13, No. 3, 1990.

8. Crassidis, J. L. and Markley, F. L., "Attitude Estimation Using Modified Rodrigues Parameters," Proceedings of the Flight Mechanics/ Estimation Theory Symposium, 1996.

9. de Ruiter, A. H. J., et al., "Sun Vector-Based Attitude Determination of Passively Magnetically Stabilized Spacecraft," AIAA Journal of Guidance, Control and Dynamics, Vol. 39, No. 7, 2016.
10. Mahony, R., Hamel, T., and Pflimlin, J.-M., "Nonlinear Complementary Filters on the Special Orthogonal Group," IEEE Transactions on Automatic Control, Vol. 53, No. 5, 2008.
11. Izadi, M. and Sanyal, A. K., "Rigid Body Attitude Estimation Based on the Lagrange-d’Alembert Principle," Automatica, Vol. 50, No. 10, 2014.
12. Zlotnik, D. E. and Forbes, J. R., "Nonlinear Estimator Design on the Special Orthogonal Group Using Vector Measurements Directly," IEEE Transactions on Automatic Control, Vol. 62, No. 1, 2017.
13. Crassidis, J. L., Markley, F. L., and Cheng, Y., "Survey of Nonlinear Attitude Estimation Methods," AIAA Journal of Guidance, Control, and Dynamics, Vol. 30, No. 1, 2007.
14. de Ruiter, A. H. J. and Forbes, J. R., "Discrete-Time SO(n)-Constrained Kalman Filtering," AIAA Journal of Guidance, Control and Dynamics, Vol. 40, No. 1, 2017.
15. Markley, F. L., "Attitude Error Representations for Kalman Filtering," Journal of Guidance, Control, and Dynamics, Vol. 63, No. 2, 2003.
16. Zanetti, R., et al., "Norm-Constrained Kalman Filtering," AIAA Journal of Guidance, Control, and Dynamics, Vol. 32, No. 5, 2009.
17. Forbes, J. R., de Ruiter, A. H. J., and Zlotnik, D. E., "Continuous-Time Norm-Constrained Kalman Filtering," Automatica, Vol. 50, 2014.
18. Psiaki, M. L., "Attitude-Determination Filtering via Extended Quaternion Estimation," AIAA Journal of Guidance, Control, and Dynamics, Vol. 23, No. 2, 2000.
19. Ainscough, T., et al., "Q-Method Extended Kalman Filter," AIAA Journal of Guidance, Control, and Dynamics, Vol. 38, No. 4, 2015.
20. Reynolds, R. G., "Asymptotically Optimal Attitude Filtering with Guaranteed Convergence," Journal of Guidance, Control, and Dynamics, Vol. 31, No. 1, 2008.
21. Mueller, M. W., Hehn, M., and D’Andrea, R., "Covariance Correction Step for Kalman Filtering with an Attitude," Journal of Guidance, Control, and Dynamics, Vol. 40, 2017.
22. Crassidis, J. L. and Markley, F. L., "Unscented Filtering for Spacecraft Attitude Estimation," AIAA Journal of Guidance, Control, and Dynamics, Vol. 26, No. 4, 2003.
23. Cheng, Y. and Crassidis, J. L., "Particle Filtering for Attitude Estimation Using a Minimal Local-Error Representation," AIAA Journal of Guidance, Control, and Dynamics, Vol. 33, No. 4, 2010.
24. Bonnabel, S., "Left-Invariant Extended Kalman Filter and Attitude Estimation," Proceedings of the 46th IEEE Conference on Decision and Control, 2007.
25. Bonnabel, S., Martin, P., and Salaün, E., "Invariant Extended Kalman Filter: Theory and Application to a Velocity-Aided Attitude Estimation Problem," Proceedings of Joint 48th IEEE CDC and 28th CCC, 2009.
26. Martin, P. and Salaün, E., "Invariant Observers for Attitude and Heading Estimation from Low-Cost Inertial and Magnetic Sensors," Proceedings of the 48th IEEE CDC, 2009.
27. Bonnabel, S., Martin, P., and Rouchon, P., "Symmetry-Preserving Observers," IEEE Transactions on Automatic Control, Vol. 53, No. 11, 2008.
28. Barczyk, M. and Lynch, A. F., "Invariant Observer Design for a Helicopter UAV Aided Inertial Navigation System," IEEE Transactions on Control Systems Technology, Vol. 21, No. 3, 2013.
29. Barrau, A. and Bonnabel, S., "Intrinsic Filtering on Lie Groups with Applications to Attitude Estimation," IEEE Transactions on Automatic Control, Vol. 60, No. 2, 2015.
30. Altmann, S., Rotations, Quaternions, and Double Groups, Oxford Science Publication, Clarendon, Oxford, 1986.
31. Crassidis, J. L. and Junkins, J. L., Optimal Estimation of Dynamic Systems, CRC Press, Boca Raton, FL, 2nd ed., 2012.
32. Gai, E., et al., "Star-Sensor-Based Satellite Attitude/Attitude Rate Estimator," Journal of Guidance, Control, and Dynamics, Vol. 8, No. 5, 1985.
33. Thébault, E., et al., "International Geomagnetic Reference Field: the 12th Generation," Earth, Planets and Space, Vol. 67, No. 79, 2015.
